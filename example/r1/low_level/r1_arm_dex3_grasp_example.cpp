#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
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

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& new_data) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_;
  }

 private:
  std::shared_ptr<T> data_;
  mutable std::shared_mutex mutex_;
};

constexpr int R1_NUM_MOTOR = 26;
constexpr int DEX3_MOTOR_MAX = 7;
constexpr int DEX3_SENSOR_MAX = 9;
constexpr float CONTROL_DT_SECONDS = 0.02f;

struct MotorState {
  std::array<float, R1_NUM_MOTOR> q = {};
  std::array<float, R1_NUM_MOTOR> dq = {};
};

const std::array<float, R1_NUM_MOTOR> Kp{
    200, 200, 200, 200, 200, 200,  // legs
    200, 200, 200, 200, 200, 200,  // legs
    300, 300,                      // waist
    100, 100, 100, 100, 50,        // left arm
    100, 100, 100, 100, 50,        // right arm
    50, 10                         // head
};

const std::array<float, R1_NUM_MOTOR> Kd{
    3, 3, 3, 3, 3, 3,      // legs
    3, 3, 3, 3, 3, 3,      // legs
    5, 5,                  // waist
    2, 2, 2, 2, 2,         // left arm
    2, 2, 2, 2, 2,         // right arm
    2, 0.1                 // head
};

const std::array<int, R1_NUM_MOTOR> joint_idx_in_idl{
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

struct Dex3Limits {
  std::array<float, DEX3_MOTOR_MAX> min;
  std::array<float, DEX3_MOTOR_MAX> max;
};

const Dex3Limits kLeftDex3Limits{
    {-1.05f, -0.724f, 0.0f, -1.57f, -1.75f, -1.57f, -1.75f},
    {1.05f, 1.05f, 1.75f, 0.0f, 0.0f, 0.0f, 0.0f}};

const Dex3Limits kRightDex3Limits{
    {-1.05f, -1.05f, -1.75f, 0.0f, 0.0f, 0.0f, 0.0f},
    {1.05f, 0.742f, 0.0f, 1.57f, 1.75f, 1.57f, 1.75f}};

struct RISMode {
  uint8_t id : 4;
  uint8_t status : 3;
  uint8_t timeout : 1;
};

enum class Side {
  LEFT,
  RIGHT
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
  ratio = std::clamp(ratio, 0.0f, 1.0f);
  return ratio * ratio * (3.0f - 2.0f * ratio);
}

std::string SideToString(Side side) {
  return side == Side::RIGHT ? "RIGHT" : "LEFT";
}

bool IsArmJointForSide(int joint_index, Side side) {
  if (side == Side::RIGHT) {
    return joint_index >= RightShoulderPitch && joint_index <= RightWristRoll;
  }
  return joint_index >= LeftShoulderPitch && joint_index <= LeftWristRoll;
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

class R1ArmDex3GraspExample {
 public:
  R1ArmDex3GraspExample(const std::string& network_interface, Side side)
      : side_(side), mode_pr_(Mode::PR), mode_machine_(0) {
    std::cout << "Initializing DDS on network interface: " << network_interface << std::endl;
    ChannelFactory::Instance()->Init(0, network_interface);
    InitR1LowLevel();
    InitDex3Hand();
  }

  void ReleaseMotionMode() {
    std::cout << "\nChecking active motion-control mode..." << std::endl;
    motion_switcher_client_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_client_->SetTimeout(5.0f);
    motion_switcher_client_->Init();

    std::string form;
    std::string name;
    while (motion_switcher_client_->CheckMode(form, name), !name.empty()) {
      std::cout << "Active motion mode detected: " << name << std::endl;
      if (motion_switcher_client_->ReleaseMode()) {
        std::cout << "Failed to switch to Release Mode." << std::endl;
      }
      sleep(2);
    }
    std::cout << "No active motion mode detected, or already released." << std::endl;
  }

  bool WaitForR1State(double timeout_seconds = 5.0) {
    std::cout << "\nWaiting for R1 lowstate..." << std::endl;
    const auto start_time = std::chrono::steady_clock::now();

    while (true) {
      if (motor_state_buffer_.GetData()) {
        std::cout << "Received R1 lowstate." << std::endl;
        return true;
      }

      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start_time).count();
      if (elapsed > timeout_seconds) {
        std::cout << "ERROR: Timed out waiting for R1 lowstate." << std::endl;
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void PrintCurrentArmState() const {
    const auto state = motor_state_buffer_.GetData();
    if (!state) {
      std::cout << "No R1 motor state available." << std::endl;
      return;
    }

    std::cout << "\nCurrent " << SideToString(side_) << " arm state:" << std::endl;
    if (side_ == Side::RIGHT) {
      PrintJoint("RightShoulderPitch", RightShoulderPitch, *state);
      PrintJoint("RightShoulderRoll ", RightShoulderRoll, *state);
      PrintJoint("RightShoulderYaw  ", RightShoulderYaw, *state);
      PrintJoint("RightElbow        ", RightElbow, *state);
      PrintJoint("RightWristRoll    ", RightWristRoll, *state);
    } else {
      PrintJoint("LeftShoulderPitch ", LeftShoulderPitch, *state);
      PrintJoint("LeftShoulderRoll  ", LeftShoulderRoll, *state);
      PrintJoint("LeftShoulderYaw   ", LeftShoulderYaw, *state);
      PrintJoint("LeftElbow         ", LeftElbow, *state);
      PrintJoint("LeftWristRoll     ", LeftWristRoll, *state);
    }
  }

  void RunDemoSequence() {
    const auto current_state_ptr = motor_state_buffer_.GetData();
    if (!current_state_ptr) {
      std::cout << "ERROR: No motor state. Cannot run demo." << std::endl;
      return;
    }

    const MotorState start_state = *current_state_ptr;
    const std::array<float, R1_NUM_MOTOR> start_pose = start_state.q;
    std::array<float, R1_NUM_MOTOR> raised_pose = start_pose;

    if (side_ == Side::RIGHT) {
      raised_pose[RightShoulderPitch] = start_pose[RightShoulderPitch] - 0.80f;
      raised_pose[RightShoulderRoll] = start_pose[RightShoulderRoll] - 0.30f;
      raised_pose[RightShoulderYaw] = start_pose[RightShoulderYaw];
      raised_pose[RightElbow] = start_pose[RightElbow] + 0.75f;
      raised_pose[RightWristRoll] = start_pose[RightWristRoll];
    } else {
      raised_pose[LeftShoulderPitch] = start_pose[LeftShoulderPitch] - 0.80f;
      raised_pose[LeftShoulderRoll] = start_pose[LeftShoulderRoll] + 0.30f;
      raised_pose[LeftShoulderYaw] = start_pose[LeftShoulderYaw];
      raised_pose[LeftElbow] = start_pose[LeftElbow] + 0.75f;
      raised_pose[LeftWristRoll] = start_pose[LeftWristRoll];
    }

    std::cout << "\nDemo sequence: arm up -> hand open -> hand close -> hand open -> arm down" << std::endl;
    MoveArmBetweenPoses(start_pose, raised_pose, 2.5f);
    HoldArmPose(raised_pose, 0.5f);

    OpenHand(1.2f, &raised_pose);
    HoldArmPose(raised_pose, 0.5f);

    CloseHand(1.2f, &raised_pose);
    HoldArmPose(raised_pose, 0.5f);

    OpenHand(1.2f, &raised_pose);
    HoldArmPose(raised_pose, 0.5f);

    MoveArmBetweenPoses(raised_pose, start_pose, 4.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    StopHand();
    std::cout << "\nDemo complete." << std::endl;
  }

 private:
  Side side_;
  Mode mode_pr_;
  std::atomic<uint8_t> mode_machine_;

  DataBuffer<MotorState> motor_state_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelPublisherPtr<HandCmd_> handcmd_publisher_;
  ChannelSubscriberPtr<HandState_> handstate_subscriber_;

  HandCmd_ hand_cmd_msg_;
  HandState_ hand_state_msg_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_client_;

  void InitR1LowLevel() {
    std::cout << "Initializing R1 low-level publisher/subscriber..." << std::endl;
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        std::bind(&R1ArmDex3GraspExample::LowStateHandler, this, std::placeholders::_1), 1);
  }

  void InitDex3Hand() {
    const std::string hand_base_topic = side_ == Side::RIGHT ? "rt/dex3/right" : "rt/dex3/left";
    const std::string hand_state_topic =
        side_ == Side::RIGHT ? "rt/lf/dex3/right/state" : "rt/lf/dex3/left/state";

    std::cout << "Initializing DEX3 hand publisher/subscriber..." << std::endl;
    std::cout << "Hand command topic: " << hand_base_topic << "/cmd" << std::endl;
    std::cout << "Hand state topic:   " << hand_state_topic << std::endl;

    handcmd_publisher_.reset(new ChannelPublisher<HandCmd_>(hand_base_topic + "/cmd"));
    handstate_subscriber_.reset(new ChannelSubscriber<HandState_>(hand_state_topic));
    handcmd_publisher_->InitChannel();
    handstate_subscriber_->InitChannel(
        std::bind(&R1ArmDex3GraspExample::HandStateHandler, this, std::placeholders::_1), 1);

    hand_cmd_msg_.motor_cmd().resize(DEX3_MOTOR_MAX);
    hand_state_msg_.motor_state().resize(DEX3_MOTOR_MAX);
    hand_state_msg_.press_sensor_state().resize(DEX3_SENSOR_MAX);
  }

  void LowStateHandler(const void* message) {
    LowState_ low_state = *(const LowState_*)message;
    if (low_state.crc() != Crc32Core((uint32_t*)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] R1 LowState CRC Error" << std::endl;
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

  void HandStateHandler(const void* message) {
    hand_state_msg_ = *(const HandState_*)message;
  }

  void PrintJoint(const std::string& name, int joint_index, const MotorState& state) const {
    std::cout << "  " << name << " q=" << state.q[joint_index] << " dq=" << state.dq[joint_index]
              << std::endl;
  }

  void MoveArmBetweenPoses(const std::array<float, R1_NUM_MOTOR>& start_pose,
                           const std::array<float, R1_NUM_MOTOR>& target_pose,
                           float duration_seconds) {
    const int steps = std::max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    std::cout << "\nMoving " << SideToString(side_) << " arm over " << duration_seconds << " seconds..."
              << std::endl;

    for (int step = 0; step <= steps; ++step) {
      const float linear_ratio =
          std::clamp(static_cast<float>(step) / static_cast<float>(steps), 0.0f, 1.0f);
      const float ratio = SmoothStep(linear_ratio);
      std::array<float, R1_NUM_MOTOR> command_pose = start_pose;

      for (int joint = 0; joint < R1_NUM_MOTOR; ++joint) {
        if (IsArmJointForSide(joint, side_)) {
          command_pose[joint] = Lerp(start_pose[joint], target_pose[joint], ratio);
        }
      }

      PublishR1BodyPose(command_pose);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
    }

    std::cout << "Arm movement finished." << std::endl;
  }

  void HoldArmPose(const std::array<float, R1_NUM_MOTOR>& hold_pose, float duration_seconds) {
    const int steps = std::max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    for (int step = 0; step < steps; ++step) {
      PublishR1BodyPose(hold_pose);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
    }
  }

  void PublishR1BodyPose(const std::array<float, R1_NUM_MOTOR>& q_target) {
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

  std::array<float, DEX3_MOTOR_MAX> OpenHandPose() const {
    return {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  }

  std::array<float, DEX3_MOTOR_MAX> ClosedHandPose(float ratio) const {
    ratio = std::clamp(ratio, 0.0f, 1.0f);
    const Dex3Limits& limits = side_ == Side::LEFT ? kLeftDex3Limits : kRightDex3Limits;
    std::array<float, DEX3_MOTOR_MAX> pose{};

    for (int i = 0; i < DEX3_MOTOR_MAX; ++i) {
      pose[i] = Lerp(limits.min[i], limits.max[i], ratio);
    }

    return pose;
  }

  void OpenHand(float duration_seconds,
                const std::array<float, R1_NUM_MOTOR>* arm_hold_pose = nullptr) {
    std::cout << "\nOpening DEX3 hand..." << std::endl;
    SendHandPoseForDuration(OpenHandPose(), duration_seconds, arm_hold_pose);
  }

  void CloseHand(float duration_seconds,
                 const std::array<float, R1_NUM_MOTOR>* arm_hold_pose = nullptr) {
    std::cout << "\nClosing DEX3 hand..." << std::endl;
    SendHandPoseForDuration(ClosedHandPose(0.85f), duration_seconds, arm_hold_pose);
  }

  void SendHandPoseForDuration(const std::array<float, DEX3_MOTOR_MAX>& pose,
                               float duration_seconds,
                               const std::array<float, R1_NUM_MOTOR>* arm_hold_pose) {
    const int steps = std::max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));

    for (int step = 0; step < steps; ++step) {
      if (arm_hold_pose) {
        PublishR1BodyPose(*arm_hold_pose);
      }
      SendHandPoseOnce(pose, false);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
    }
  }

  void SendHandPoseOnce(const std::array<float, DEX3_MOTOR_MAX>& pose, bool timeout) {
    for (int i = 0; i < DEX3_MOTOR_MAX; ++i) {
      hand_cmd_msg_.motor_cmd()[i].mode(MakeHandMode(i, timeout));
      hand_cmd_msg_.motor_cmd()[i].tau(0.0f);
      hand_cmd_msg_.motor_cmd()[i].q(pose[i]);
      hand_cmd_msg_.motor_cmd()[i].dq(0.0f);
      hand_cmd_msg_.motor_cmd()[i].kp(timeout ? 0.0f : 1.5f);
      hand_cmd_msg_.motor_cmd()[i].kd(timeout ? 0.0f : 0.1f);
    }

    handcmd_publisher_->Write(hand_cmd_msg_);
  }

  void StopHand() {
    std::cout << "\nStopping DEX3 hand motors..." << std::endl;
    SendHandPoseOnce(OpenHandPose(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "DEX3 hand stop command sent." << std::endl;
  }
};

void PrintUsage(const char* program_name) {
  std::cout << "Usage:" << std::endl;
  std::cout << "  " << program_name << " <network_interface> R" << std::endl;
  std::cout << "  " << program_name << " <network_interface> L" << std::endl;
  std::cout << std::endl;
  std::cout << "Example:" << std::endl;
  std::cout << "  " << program_name << " enp0s31f6 R" << std::endl;
}

bool ParseSide(const std::string& side_arg, Side* side) {
  if (side_arg.size() != 1) {
    return false;
  }

  const char side_char = static_cast<char>(std::toupper(static_cast<unsigned char>(side_arg[0])));
  if (side_char == 'R') {
    *side = Side::RIGHT;
    return true;
  }
  if (side_char == 'L') {
    *side = Side::LEFT;
    return true;
  }

  return false;
}

int main(int argc, char const* argv[]) {
  if (argc != 3) {
    PrintUsage(argv[0]);
    return 0;
  }

  const std::string network_interface = argv[1];
  Side side = Side::RIGHT;
  if (!ParseSide(argv[2], &side)) {
    std::cout << "Invalid side argument: " << argv[2] << std::endl;
    PrintUsage(argv[0]);
    return -1;
  }

  std::cout << "================================" << std::endl;
  std::cout << " R1 Arm with DEX3 grasp example " << std::endl;
  std::cout << "================================" << std::endl;
  std::cout << "Network interface: " << network_interface << std::endl;
  std::cout << "Selected side:     " << SideToString(side) << std::endl;
  std::cout << std::endl;
  std::cout << "Safety check before continuing:" << std::endl;
  std::cout << "  - R1 is powered, stable, and has free arm/hand clearance." << std::endl;
  std::cout << "  - The selected DEX3 hand is installed on the selected arm." << std::endl;
  std::cout << "  - You can stop the robot immediately if movement is unexpected." << std::endl;
  std::cout << "\nPress ENTER once to initialize and run the full automatic sequence." << std::endl;
  std::cin.get();

  R1ArmDex3GraspExample example(network_interface, side);
  example.ReleaseMotionMode();

  if (!example.WaitForR1State(5.0)) {
    std::cout << "Could not receive R1 state. Exiting." << std::endl;
    return -1;
  }

  example.PrintCurrentArmState();
  example.RunDemoSequence();

  std::cout << "\nProgram finished." << std::endl;
  return 0;
}
