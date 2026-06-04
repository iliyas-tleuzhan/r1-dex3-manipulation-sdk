#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

constexpr int DEX3_MOTOR_MAX = 7;

enum class Side {
  LEFT,
  RIGHT
};

struct HandJointLabel {
  const char* urdf_name;
  const char* joint_name;
};

const std::array<HandJointLabel, DEX3_MOTOR_MAX> LEFT_HAND_LABELS{{
    {"left_hand_zero", "thumb_0"},
    {"left_hand_one", "thumb_1"},
    {"left_hand_two", "thumb_2"},
    {"left_hand_five", "middle_0"},
    {"left_hand_six", "middle_1"},
    {"left_hand_three", "index_0"},
    {"left_hand_four", "index_1"},
}};

const std::array<HandJointLabel, DEX3_MOTOR_MAX> RIGHT_HAND_LABELS{{
    {"right_hand_zero", "thumb_0"},
    {"right_hand_one", "thumb_1"},
    {"right_hand_two", "thumb_2"},
    {"right_hand_three", "middle_0"},
    {"right_hand_four", "middle_1"},
    {"right_hand_five", "index_0"},
    {"right_hand_six", "index_1"},
}};

std::atomic<bool> running{true};

void SignalHandler(int) {
  running = false;
}

std::string SideToString(Side side) {
  return side == Side::RIGHT ? "RIGHT" : "LEFT";
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

std::string HandStateTopic(Side side) {
  return side == Side::RIGHT ? "rt/lf/dex3/right/state" : "rt/lf/dex3/left/state";
}

const std::array<HandJointLabel, DEX3_MOTOR_MAX>& LabelsForSide(Side side) {
  return side == Side::RIGHT ? RIGHT_HAND_LABELS : LEFT_HAND_LABELS;
}

class Dex3HandPositionMonitor {
 public:
  Dex3HandPositionMonitor(const std::string& network_interface, Side side)
      : side_(side), received_state_(false) {
    ChannelFactory::Instance()->Init(0, network_interface);

    const std::string topic = HandStateTopic(side_);
    std::cout << "Subscribing to DEX3 hand state topic: " << topic << std::endl;
    std::cout << "Read-only monitor: no HandCmd publisher is created, so the hand is not stiffened or locked."
              << std::endl;

    handstate_subscriber_.reset(new ChannelSubscriber<HandState_>(topic));
    handstate_subscriber_->InitChannel(
        std::bind(&Dex3HandPositionMonitor::HandStateHandler, this, std::placeholders::_1), 1);
  }

  void Run() {
    while (running) {
      PrintState();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nStopped DEX3 hand position monitor." << std::endl;
  }

 private:
  Side side_;
  std::mutex mutex_;
  HandState_ latest_state_;
  bool received_state_;
  ChannelSubscriberPtr<HandState_> handstate_subscriber_;

  void HandStateHandler(const void* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_ = *(const HandState_*)message;
    received_state_ = true;
  }

  void PrintState() {
    HandState_ state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!received_state_) {
        std::cout << "\rWaiting for " << SideToString(side_) << " DEX3 hand state..." << std::flush;
        return;
      }
      state = latest_state_;
    }

    std::cout << "\033[2J\033[H";
    std::cout << "DEX3 " << SideToString(side_) << " hand current motor positions" << std::endl;
    std::cout << "Topic: " << HandStateTopic(side_) << std::endl;
    std::cout << "This program only subscribes to state. It does not command, stiffen, or lock motors."
              << std::endl;
    std::cout << std::endl;

    std::cout << std::left << std::setw(5) << "IDL" << std::setw(20) << "URDF label" << std::setw(14)
              << "Joint" << std::right << std::setw(12) << "q(rad)" << std::setw(12) << "dq" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

    const auto& labels = LabelsForSide(side_);
    const auto& motor_state = state.motor_state();
    for (int i = 0; i < DEX3_MOTOR_MAX; ++i) {
      if (static_cast<size_t>(i) >= motor_state.size()) {
        std::cout << std::left << std::setw(5) << i << std::setw(20) << labels[i].urdf_name
                  << std::setw(14) << labels[i].joint_name << std::right << std::setw(12) << "n/a"
                  << std::setw(12) << "n/a" << std::endl;
        continue;
      }

      std::cout << std::left << std::setw(5) << i << std::setw(20) << labels[i].urdf_name
                << std::setw(14) << labels[i].joint_name << std::right << std::fixed
                << std::setprecision(5) << std::setw(12) << motor_state[i].q() << std::setw(12)
                << motor_state[i].dq() << std::endl;
    }
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

int main(int argc, char const* argv[]) {
  if (argc != 3) {
    PrintUsage(argv[0]);
    return 0;
  }

  Side side = Side::RIGHT;
  if (!ParseSide(argv[2], &side)) {
    std::cout << "Invalid side argument: " << argv[2] << std::endl;
    PrintUsage(argv[0]);
    return -1;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  Dex3HandPositionMonitor monitor(argv[1], side);
  monitor.Run();
  return 0;
}
