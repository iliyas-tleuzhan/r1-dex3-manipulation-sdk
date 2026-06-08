#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

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
constexpr float CONTROL_DT_SECONDS = 0.02f;
constexpr float PI = 3.14159265358979323846f;

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

const std::array<float, DEX3_MOTOR_MAX> kLeftDex3OpenPose{
    -0.07746f, -0.59442f, -0.04216f, -0.03199f, -0.01694f, -0.02221f, -0.01527f};

const std::array<float, DEX3_MOTOR_MAX> kRightDex3OpenPose{
    -0.08564f, 0.57882f, -0.02465f, -0.01603f, -0.04005f, -0.00692f, -0.06273f};

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

class R1ArmWaveHandExample {
 public:
  R1ArmWaveHandExample(const std::string& network_interface, Side side)
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

  void RunWaveSequence() {
    const auto current_state_ptr = motor_state_buffer_.GetData();
    if (!current_state_ptr) {
      std::cout << "ERROR: No motor state. Cannot run wave." << std::endl;
      return;
    }

    const std::array<float, R1_NUM_MOTOR> start_pose = current_state_ptr->q;
    std::array<float, R1_NUM_MOTOR> raised_pose = start_pose;

    if (side_ == Side::RIGHT) {
      raised_pose[RightShoulderPitch] = start_pose[RightShoulderPitch] - 0.90f;
      raised_pose[RightShoulderRoll] = start_pose[RightShoulderRoll] - 0.35f;
      raised_pose[RightShoulderYaw] = start_pose[RightShoulderYaw];
      raised_pose[RightElbow] = start_pose[RightElbow] - 0.85f;
      raised_pose[RightWristRoll] = start_pose[RightWristRoll];
    } else {
      raised_pose[LeftShoulderPitch] = start_pose[LeftShoulderPitch] - 0.90f;
      raised_pose[LeftShoulderRoll] = start_pose[LeftShoulderRoll] + 0.35f;
      raised_pose[LeftShoulderYaw] = start_pose[LeftShoulderYaw];
      raised_pose[LeftElbow] = start_pose[LeftElbow] - 0.85f;
      raised_pose[LeftWristRoll] = start_pose[LeftWristRoll];
    }

    std::cout << "\nWave sequence: raise arm -> open hand -> wave -> lower arm" << std::endl;
    MoveArmBetweenPoses(start_pose, raised_pose, 2.5f);
    SendOpenHandForDuration(0.8f, &raised_pose);
    HoldArmPose(raised_pose, 0.4f);

    WaveRaisedArm(raised_pose, 4, 0.7f);

    HoldArmPose(raised_pose, 0.4f);
    MoveArmBetweenPoses(raised_pose, start_pose, 3.0f);
    StopHand();

    std::cout << "\nWave complete." << std::endl;
  }

 private:
  Side side_;
  Mode mode_pr_;
  std::atomic<uint8_t> mode_machine_;

  DataBuffer<MotorState> motor_state_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelPublisherPtr<HandCmd_> handcmd_publisher_;

  HandCmd_ hand_cmd_msg_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_client_;

  void InitR1LowLevel() {
    std::cout << "Initializing R1 low-level publisher/subscriber..." << std::endl;
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        std::bind(&R1ArmWaveHandExample::LowStateHandler, this, std::placeholders::_1), 1);
  }

  void InitDex3Hand() {
    const std::string hand_base_topic = side_ == Side::RIGHT ? "rt/dex3/right" : "rt/dex3/left";

    std::cout << "Initializing DEX3 hand publisher..." << std::endl;
    std::cout << "Hand command topic: " << hand_base_topic << "/cmd" << std::endl;

    handcmd_publisher_.reset(new ChannelPublisher<HandCmd_>(hand_base_topic + "/cmd"));
    handcmd_publisher_->InitChannel();
    hand_cmd_msg_.motor_cmd().resize(DEX3_MOTOR_MAX);
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

      for (int joint = ArmFirstJoint(); joint <= ArmLastJoint(); ++joint) {
        command_pose[joint] = Lerp(start_pose[joint], target_pose[joint], ratio);
      }

      PublishR1BodyPose(command_pose);
      SendOpenHandOnce(false);
      SleepControlDt();
    }
  }

  void HoldArmPose(const std::array<float, R1_NUM_MOTOR>& hold_pose, float duration_seconds) {
    const int steps = std::max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    for (int step = 0; step < steps; ++step) {
      PublishR1BodyPose(hold_pose);
      SendOpenHandOnce(false);
      SleepControlDt();
    }
  }

  void WaveRaisedArm(const std::array<float, R1_NUM_MOTOR>& raised_pose,
                     int wave_count,
                     float seconds_per_wave) {
    std::cout << "\nWaving " << SideToString(side_) << " hand..." << std::endl;
    const int steps_per_wave = std::max(2, static_cast<int>(seconds_per_wave / CONTROL_DT_SECONDS));
    const int total_steps = std::max(1, wave_count * steps_per_wave);
    const int yaw_joint = side_ == Side::RIGHT ? RightShoulderYaw : LeftShoulderYaw;
    const int wrist_joint = side_ == Side::RIGHT ? RightWristRoll : LeftWristRoll;

    for (int step = 0; step <= total_steps; ++step) {
      const float progress = static_cast<float>(step) / static_cast<float>(total_steps);
      const float offset_ratio = std::sin(2.0f * PI * static_cast<float>(wave_count) * progress);

      std::array<float, R1_NUM_MOTOR> command_pose = raised_pose;
      command_pose[yaw_joint] = raised_pose[yaw_joint] + 0.30f * offset_ratio;
      command_pose[wrist_joint] = raised_pose[wrist_joint] + 0.45f * offset_ratio;

      PublishR1BodyPose(command_pose);
      SendOpenHandOnce(false);
      SleepControlDt();
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

  void SendOpenHandForDuration(float duration_seconds,
                               const std::array<float, R1_NUM_MOTOR>* arm_hold_pose) {
    std::cout << "\nOpening DEX3 hand..." << std::endl;
    const int steps = std::max(1, static_cast<int>(duration_seconds / CONTROL_DT_SECONDS));
    for (int step = 0; step < steps; ++step) {
      if (arm_hold_pose) {
        PublishR1BodyPose(*arm_hold_pose);
      }
      SendOpenHandOnce(false);
      SleepControlDt();
    }
  }

  void SendOpenHandOnce(bool timeout) {
    const std::array<float, DEX3_MOTOR_MAX>& pose =
        side_ == Side::LEFT ? kLeftDex3OpenPose : kRightDex3OpenPose;

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
    SendOpenHandOnce(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "DEX3 hand stop command sent." << std::endl;
  }

  int ArmFirstJoint() const {
    return side_ == Side::RIGHT ? RightShoulderPitch : LeftShoulderPitch;
  }

  int ArmLastJoint() const {
    return side_ == Side::RIGHT ? RightWristRoll : LeftWristRoll;
  }

  void SleepControlDt() const {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(CONTROL_DT_SECONDS * 1000.0f)));
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

  std::cout << "==========================" << std::endl;
  std::cout << " R1 Arm DEX3 wave example " << std::endl;
  std::cout << "==========================" << std::endl;
  std::cout << "Network interface: " << network_interface << std::endl;
  std::cout << "Selected side:     " << SideToString(side) << std::endl;
  std::cout << std::endl;
  std::cout << "Safety check before continuing:" << std::endl;
  std::cout << "  - R1 is powered, stable, and has free clearance above and beside the selected arm." << std::endl;
  std::cout << "  - The selected DEX3 hand is installed and clear of objects/people." << std::endl;
  std::cout << "  - You can stop the robot immediately if movement is unexpected." << std::endl;
  std::cout << "\nPress ENTER once to initialize and run the full automatic wave sequence." << std::endl;
  std::cin.get();

  R1ArmWaveHandExample example(network_interface, side);
  example.ReleaseMotionMode();

  if (!example.WaitForR1State(5.0)) {
    std::cout << "Could not receive R1 state. Exiting." << std::endl;
    return -1;
  }

  example.PrintCurrentArmState();
  example.RunWaveSequence();

  std::cout << "\nProgram finished." << std::endl;
  return 0;
}
