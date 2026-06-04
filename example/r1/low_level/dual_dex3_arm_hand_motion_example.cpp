#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace std;
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

static const string HG_CMD_TOPIC = "rt/lowcmd";
static const string HG_STATE_TOPIC = "rt/lowstate";
static const string LEFT_DEX3_CMD_TOPIC = "rt/dex3/left/cmd";
static const string RIGHT_DEX3_CMD_TOPIC = "rt/dex3/right/cmd";
static const string LEFT_DEX3_STATE_TOPIC = "rt/lf/dex3/left/state";
static const string RIGHT_DEX3_STATE_TOPIC = "rt/lf/dex3/right/state";

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& new_data) {
    unique_lock<shared_mutex> lock(mutex_);
    data_ = make_shared<T>(new_data);
  }

  shared_ptr<const T> GetData() const {
    shared_lock<shared_mutex> lock(mutex_);
    return data_;
  }

 private:
  shared_ptr<T> data_;
  mutable shared_mutex mutex_;
};

constexpr int R1_NUM_MOTOR = 26;
constexpr int DEX3_MOTOR_MAX = 7;
constexpr int DEX3_SENSOR_MAX = 9;
constexpr float CONTROL_DT_SECONDS = 0.02f;

struct MotorState {
  array<float, R1_NUM_MOTOR> q = {};
  array<float, R1_NUM_MOTOR> dq = {};
};

const array<float, R1_NUM_MOTOR> Kp{
    200, 200, 200, 200, 200, 200,  // legs
    200, 200, 200, 200, 200, 200,  // legs
    300, 300,                      // waist
    100, 100, 100, 100, 50,        // left arm
    100, 100, 100, 100, 50,        // right arm
    50, 10                         // head
};

const array<float, R1_NUM_MOTOR> Kd{
    3, 3, 3, 3, 3, 3,      // legs
    3, 3, 3, 3, 3, 3,      // legs
    5, 5,                  // waist
    2, 2, 2, 2, 2,         // left arm
    2, 2, 2, 2, 2,         // right arm
    2, 0.1                 // head
};

const array<int, R1_NUM_MOTOR> joint_idx_in_idl{
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30
};

enum class Mode : uint8_t {
  PR = 0,
  AB = 1
};

enum R1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleRoll = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleRoll = 11,
  WaistRoll = 12,
  WaistYaw = 13,
  LeftShoulderPitch = 14,
  LeftShoulderRoll = 15,
  LeftShoulderYaw = 16,
  LeftElbow = 17,
  LeftWristRoll = 18,
  RightShoulderPitch = 19,
  RightShoulderRoll = 20,
  RightShoulderYaw = 21,
  RightElbow = 22,
  RightWristRoll = 23,
  HeadPitch = 24,
  HeadYaw = 25,
};

const array<float, DEX3_MOTOR_MAX> kLeftDex3OpenPose{
    -0.07746f, -0.59442f, -0.04216f, -0.03199f, -0.01694f, -0.02221f, -0.01527f};

const array<float, DEX3_MOTOR_MAX> kRightDex3OpenPose{
    -0.08564f, 0.57882f, -0.02465f, -0.01603f, -0.04005f, -0.00692f, -0.06273f};

const array<float, DEX3_MOTOR_MAX> kLeftDex3ClosedPose{
    -0.07639f, 1.01445f, 1.51530f, -1.59280f, -1.80383f, -1.59922f, -1.79961f};

const array<float, DEX3_MOTOR_MAX> kRightDex3ClosedPose{
    -0.08551f, -1.00146f, -1.52876f, 1.55184f, 1.71793f, 1.54252f, 1.72397f};

struct RISMode {
  uint8_t id : 4;
  uint8_t status : 3;
  uint8_t timeout : 1;
};

inline uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  const uint32_t dw_polynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1;
        crc32 ^= dw_polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) {
        crc32 ^= dw_polynomial;
      }
      xbit >>= 1;
    }
  }

  return crc32;
}

float Lerp(float start, float target, float ratio) {
  return start * (1.0f - ratio) + target * ratio;
}

float SmoothStep(float ratio) {
  ratio = clamp(ratio, 0.0f, 1.0f);
  return ratio * ratio * (3.0f - 2.0f * ratio);
}

uint8_t MakeHandMode(int motor_id, bool timeout) {
  RISMode ris_mode{};
  ris_mode.id = static_cast<uint8_t>(motor_id);
  ris_mode.status = 0x01;
  ris_mode.timeout = timeout ? 0x01 : 0x00;

  uint8_t mode = 0;
  mode |= (ris_mode.id & 0x0F);
  mode |= (ris_mode.status & 0x07) << 4;
  mode |= (ris_mode.timeout & 0x01) << 7;
  return mode;
}

class DualDex3ArmHandMotionExample {
 public:
  explicit DualDex3ArmHandMotionExample(const string& network_interface)
      : mode_pr_(Mode::PR), mode_machine_(0) {
    cout << "Initializing DDS on network interface: " << network_interface << endl;
    ChannelFactory::Instance()->Init(0, network_interface);
    InitLowLevel();
    InitDex3Hands();
  }

  void ReleaseMotionMode() {
    cout << "\nChecking active motion-control mode..." << endl;
    motion_switcher_client_ = make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_client_->SetTimeout(5.0f);
    motion_switcher_client_->Init();

    string form;
    string name;
    while (motion_switcher_client_->CheckMode(form, name), !name.empty()) {
      cout << "Active motion mode detected: " << name << endl;
      if (motion_switcher_client_->ReleaseMode()) {
        cout << "Failed to switch to Release Mode." << endl;
      }
      sleep(2);
    }
    cout << "No active motion mode detected, or already released." << endl;
  }

  bool WaitForR1State(double timeout_seconds = 5.0) {
    cout << "\nWaiting for R1 lowstate..." << endl;
    const auto start_time = chrono::steady_clock::now();

    while (true) {
      if (motor_state_buffer_.GetData()) {
        cout << "Received R1 lowstate." << endl;
        return true;
      }

      const auto now = chrono::steady_clock::now();
      const double elapsed = chrono::duration<double>(now - start_time).count();
      if (elapsed > timeout_seconds) {
        cout << "ERROR: Timed out waiting for R1 lowstate." << endl;
        return false;
      }

      this_thread::sleep_for(chrono::milliseconds(20));
    }
  }

  void PrintCurrentArmState() const {
    const auto state = motor_state_buffer_.GetData();
    if (!state) {
      cout << "No R1 motor state available." << endl;
      return;
    }

    cout << "\nCurrent arm state:" << endl;
    PrintJoint("LeftShoulderPitch ", LeftShoulderPitch, *state);
    PrintJoint("LeftShoulderRoll  ", LeftShoulderRoll, *state);
    PrintJoint("LeftShoulderYaw   ", LeftShoulderYaw, *state);
    PrintJoint("LeftElbow         ", LeftElbow, *state);
    PrintJoint("LeftWristRoll     ", LeftWristRoll, *state);
    PrintJoint("RightShoulderPitch", RightShoulderPitch, *state);
    PrintJoint("RightShoulderRoll ", RightShoulderRoll, *state);
    PrintJoint("RightShoulderYaw  ", RightShoulderYaw, *state);
    PrintJoint("RightElbow        ", RightElbow, *state);
    PrintJoint("RightWristRoll    ", RightWristRoll, *state);
  }

  void RunDemoSequence() {
    const auto current_state_ptr = motor_state_buffer_.GetData();
    if (!current_state_ptr) {
      cout << "ERROR: No motor state. Cannot run demo." << endl;
      return;
    }

    const MotorState start_state = *current_state_ptr;
    const array<float, R1_NUM_MOTOR> start_pose = start_state.q;
    array<float, R1_NUM_MOTOR> raised_pose = start_pose;

    raised_pose[LeftShoulderPitch] = start_pose[LeftShoulderPitch] - 0.80f;
    raised_pose[LeftShoulderRoll] = start_pose[LeftShoulderRoll] + 0.30f;
    raised_pose[LeftShoulderYaw] = start_pose[LeftShoulderYaw];
    raised_pose[LeftElbow] = start_pose[LeftElbow] - 0.75f;
    raised_pose[LeftWristRoll] = start_pose[LeftWristRoll];

    raised_pose[RightShoulderPitch] = start_pose[RightShoulderPitch] - 0.80f;
    raised_pose[RightShoulderRoll] = start_pose[RightShoulderRoll] - 0.30f;
    raised_pose[RightShoulderYaw] = start_pose[RightShoulderYaw];
    raised_pose[RightElbow] = start_pose[RightElbow] - 0.75f;
    raised_pose[RightWristRoll] = start_pose[RightWristRoll];

    cout << "\nDemo sequence: both arms up -> both hands open -> close -> open -> close -> both arms down"
         << endl;

    MoveArmsBetweenPoses(start_pose, raised_pose, 2.5f);
    HoldArmPose(raised_pose, 0.5f);

    SendBothHandsPoseForDuration(kLeftDex3OpenPose, kRightDex3OpenPose,
                                 1.2f, &raised_pose, "Opening both DEX3 hands");
    HoldArmPose(raised_pose, 0.5f);

    SendBothHandsPoseForDuration(kLeftDex3ClosedPose, kRightDex3ClosedPose,
                                 1.2f, &raised_pose, "Closing both DEX3 hands");
    HoldArmPose(raised_pose, 0.5f);

    SendBothHandsPoseForDuration(kLeftDex3OpenPose, kRightDex3OpenPose,
                                 1.2f, &raised_pose, "Opening both DEX3 hands again");
    HoldArmPose(raised_pose, 0.5f);

    SendBothHandsPoseForDuration(kLeftDex3ClosedPose, kRightDex3ClosedPose,
                                 1.2f, &raised_pose, "Closing both DEX3 hands again");
    HoldArmPose(raised_pose, 0.5f);

    MoveArmsBetweenPoses(raised_pose, start_pose, 4.0f);
    SendBothHandsOnce(kLeftDex3OpenPose, kRightDex3OpenPose, false);
    HoldArmPose(start_pose, 0.5f);

    StopHands();
    cout << "\nDemo complete." << endl;
  }

 private:
  Mode mode_pr_;
  atomic<uint8_t> mode_machine_;

  DataBuffer<MotorState> motor_state_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelPublisherPtr<HandCmd_> left_handcmd_publisher_;
  ChannelPublisherPtr<HandCmd_> right_handcmd_publisher_;
  ChannelSubscriberPtr<HandState_> left_handstate_subscriber_;
  ChannelSubscriberPtr<HandState_> right_handstate_subscriber_;

  HandCmd_ left_hand_cmd_msg_;
  HandCmd_ right_hand_cmd_msg_;
  HandState_ left_hand_state_msg_;
  HandState_ right_hand_state_msg_;

  shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_client_;

  void InitLowLevel() {
    cout << "Initializing R1 low-level publisher/subscriber..." << endl;
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        bind(&DualDex3ArmHandMotionExample::LowStateHandler, this, placeholders::_1), 1);
  }

  void InitDex3Hands() {
    cout << "Initializing both DEX3 hand publishers/subscribers..." << endl;
    cout << "Left command topic:  " << LEFT_DEX3_CMD_TOPIC << endl;
    cout << "Right command topic: " << RIGHT_DEX3_CMD_TOPIC << endl;

    left_handcmd_publisher_.reset(new ChannelPublisher<HandCmd_>(LEFT_DEX3_CMD_TOPIC));
    right_handcmd_publisher_.reset(new ChannelPublisher<HandCmd_>(RIGHT_DEX3_CMD_TOPIC));
    left_handstate_subscriber_.reset(new ChannelSubscriber<HandState_>(LEFT_DEX3_STATE_TOPIC));
    right_handstate_subscriber_.reset(new ChannelSubscriber<HandState_>(RIGHT_DEX3_STATE_TOPIC));

    left_handcmd_publisher_->InitChannel();
    right_handcmd_publisher_->InitChannel();
    left_handstate_subscriber_->InitChannel(
        bind(&DualDex3ArmHandMotionExample::LeftHandStateHandler, this, placeholders::_1), 1);
    right_handstate_subscriber_->InitChannel(
        bind(&DualDex3ArmHandMotionExample::RightHandStateHandler, this, placeholders::_1), 1);

    left_hand_cmd_msg_.motor_cmd().resize(DEX3_MOTOR_MAX);
    right_hand_cmd_msg_.motor_cmd().resize(DEX3_MOTOR_MAX);
    left_hand_state_msg_.motor_state().resize(DEX3_MOTOR_MAX);
    right_hand_state_msg_.motor_state().resize(DEX3_MOTOR_MAX);
    left_hand_state_msg_.press_sensor_state().resize(DEX3_SENSOR_MAX);
    right_hand_state_msg_.press_sensor_state().resize(DEX3_SENSOR_MAX);
  }

  void LowStateHandler(const void* message) {
    LowState_ low_state = *(const LowState_*)message;
    if (low_state.crc() != Crc32Core((uint32_t*)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      cout << "[ERROR] R1 LowState CRC Error" << endl;
      return;
    }

    MotorState motor_state;
    for (int i = 0; i < R1_NUM_MOTOR; ++i) {
      const int idl_index = joint_idx_in_idl[i];
      motor_state.q.at(i) = low_state.motor_state()[idl_index].q();
      motor_state.dq.at(i) = low_state.motor_state()[idl_index].dq();
    }

    motor_state_buffer_.SetData(motor_state);
    mode_machine_.store(low_state.mode_machine());
  }

  void LeftHandStateHandler(const void* message) {
    left_hand_state_msg_ = *(const HandState_*)message;
  }

  void RightHandStateHandler(const void* message) {
    right_hand_state_msg_ = *(const HandState_*)message;
  }

  void PrintJoint(const string& name, int joint_index, const MotorState& state) const {
    cout << "  " << name << " q=" << state.q[joint_index]
         << " dq=" << state.dq[joint_index] << endl;
  }

  void MoveArmsBetweenPoses(const array<float, R1_NUM_MOTOR>& start_pose,
                            const array<float, R1_NUM_MOTOR>& target_pose,
                            float duration_seconds) {
    const int steps = max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    cout << "\nMoving both arms over " << duration_seconds << " seconds..." << endl;

    for (int step = 0; step <= steps; ++step) {
      const float linear_ratio =
          clamp(static_cast<float>(step) / static_cast<float>(steps), 0.0f, 1.0f);
      const float ratio = SmoothStep(linear_ratio);
      array<float, R1_NUM_MOTOR> command_pose = start_pose;

      for (int joint = LeftShoulderPitch; joint <= LeftWristRoll; ++joint) {
        command_pose[joint] = Lerp(start_pose[joint], target_pose[joint], ratio);
      }
      for (int joint = RightShoulderPitch; joint <= RightWristRoll; ++joint) {
        command_pose[joint] = Lerp(start_pose[joint], target_pose[joint], ratio);
      }

      PublishR1BodyPose(command_pose);
      SleepControlDt();
    }

    cout << "Arm movement finished." << endl;
  }

  void HoldArmPose(const array<float, R1_NUM_MOTOR>& hold_pose, float duration_seconds) {
    const int steps = max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    for (int step = 0; step < steps; ++step) {
      PublishR1BodyPose(hold_pose);
      SleepControlDt();
    }
  }

  void PublishR1BodyPose(const array<float, R1_NUM_MOTOR>& q_target) {
    LowCmd_ cmd;
    cmd.mode_pr() = static_cast<uint8_t>(mode_pr_);
    cmd.mode_machine() = mode_machine_.load();

    for (int i = 0; i < R1_NUM_MOTOR; ++i) {
      const int idl_index = joint_idx_in_idl[i];
      cmd.motor_cmd().at(idl_index).mode() = 1;
      cmd.motor_cmd().at(idl_index).tau() = 0.0f;
      cmd.motor_cmd().at(idl_index).q() = q_target[i];
      cmd.motor_cmd().at(idl_index).dq() = 0.0f;
      cmd.motor_cmd().at(idl_index).kp() = Kp[i];
      cmd.motor_cmd().at(idl_index).kd() = Kd[i];
    }

    cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
    lowcmd_publisher_->Write(cmd);
  }

  void SendBothHandsPoseForDuration(const array<float, DEX3_MOTOR_MAX>& left_pose,
                                    const array<float, DEX3_MOTOR_MAX>& right_pose,
                                    float duration_seconds,
                                    const array<float, R1_NUM_MOTOR>* arm_hold_pose,
                                    const string& label) {
    const int steps = max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    cout << "\n" << label << "..." << endl;

    for (int step = 0; step < steps; ++step) {
      if (arm_hold_pose) {
        PublishR1BodyPose(*arm_hold_pose);
      }
      SendBothHandsOnce(left_pose, right_pose, false);
      SleepControlDt();
    }
  }

  void SendBothHandsOnce(const array<float, DEX3_MOTOR_MAX>& left_pose,
                         const array<float, DEX3_MOTOR_MAX>& right_pose,
                         bool timeout) {
    FillHandCommand(&left_hand_cmd_msg_, left_pose, timeout);
    FillHandCommand(&right_hand_cmd_msg_, right_pose, timeout);
    left_handcmd_publisher_->Write(left_hand_cmd_msg_);
    right_handcmd_publisher_->Write(right_hand_cmd_msg_);
  }

  void FillHandCommand(HandCmd_* cmd,
                       const array<float, DEX3_MOTOR_MAX>& pose,
                       bool timeout) {
    for (int i = 0; i < DEX3_MOTOR_MAX; ++i) {
      cmd->motor_cmd()[i].mode(MakeHandMode(i, timeout));
      cmd->motor_cmd()[i].tau(0.0f);
      cmd->motor_cmd()[i].q(pose[i]);
      cmd->motor_cmd()[i].dq(0.0f);
      cmd->motor_cmd()[i].kp(timeout ? 0.0f : 1.5f);
      cmd->motor_cmd()[i].kd(timeout ? 0.0f : 0.1f);
    }
  }

  void StopHands() {
    cout << "\nStopping both DEX3 hands..." << endl;
    SendBothHandsOnce(kLeftDex3OpenPose, kRightDex3OpenPose, true);
    this_thread::sleep_for(chrono::milliseconds(500));
    cout << "DEX3 hand stop commands sent." << endl;
  }

  void SleepControlDt() const {
    this_thread::sleep_for(
        chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
  }
};

void PrintUsage(const char* program_name) {
  cout << "Usage:" << endl;
  cout << "  " << program_name << " <network_interface>" << endl;
  cout << endl;
  cout << "Example:" << endl;
  cout << "  " << program_name << " enp0s31f6" << endl;
}

int main(int argc, char const* argv[]) {
  if (argc != 2) {
    PrintUsage(argv[0]);
    return 0;
  }

  const string network_interface = argv[1];

  cout << "=========================================" << endl;
  cout << " R1 dual-arm DEX3 open/close x2 example " << endl;
  cout << "=========================================" << endl;
  cout << "Network interface: " << network_interface << endl;
  cout << endl;
  cout << "Safety check before continuing:" << endl;
  cout << "  - R1 is powered, stable, and has free clearance for both arms and hands." << endl;
  cout << "  - Both DEX3 hands are installed and clear of objects/people." << endl;
  cout << "  - You can stop the robot immediately if movement is unexpected." << endl;
  cout << "\nPress ENTER once to initialize and run the full automatic sequence." << endl;
  cin.get();

  DualDex3ArmHandMotionExample example(network_interface);
  example.ReleaseMotionMode();

  if (!example.WaitForR1State(5.0)) {
    cout << "Could not receive R1 state. Exiting." << endl;
    return -1;
  }

  example.PrintCurrentArmState();
  example.RunDemoSequence();

  cout << "\nProgram finished." << endl;
  return 0;
}
