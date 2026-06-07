# R1 DEX3-1 Manipulation SDK

SDK2-based development repo for Unitree R1 EDU humanoid manipulation with DEX3-1 dexterous hands.

This repository is focused on low-level DDS / SDK2 coding for R1 arm + DEX3-1 manipulation. It is not a general Unitree SDK2 example collection.

## Examples

`dex3_hand_position_monitor` reads DEX3 hand positions without commanding the hand:

```bash
./build/bin/dex3_hand_position_monitor <network_interface> R
./build/bin/dex3_hand_position_monitor <network_interface> L
```

This monitor is subscriber-only. It does not create a `HandCmd_` publisher and does not send motor commands, so it should not stiffen or lock the DEX3 hand while you move it by hand.

`r1_arm_dex3_grasp_example` runs a conservative automatic sequence on the selected arm and hand:

```text
arm up -> hand open -> hand close -> hand open -> arm down
```

```bash
./build/bin/r1_arm_dex3_grasp_example <network_interface> R
./build/bin/r1_arm_dex3_grasp_example <network_interface> L
```

`dual_dex3_arm_hand_motion_example` raises both arms and opens/closes both DEX3 hands twice:

```bash
./build/bin/dual_dex3_arm_hand_motion_example <network_interface>
```

`r1_arm_wave_hand_example` raises the selected arm, opens the DEX3 hand, waves with shoulder yaw and wrist roll, then lowers the arm:

```bash
./build/bin/r1_arm_wave_hand_example <network_interface> R
./build/bin/r1_arm_wave_hand_example <network_interface> L
```

Each command example takes one safety confirmation before initialization, then runs its automatic sequence without requiring ENTER between each motion.

## Safety Warning

These examples send low-level R1 joint commands and DEX3 hand motor commands. Test only with the robot in a stable, supervised setup with clear space around the selected arm and hand. Be ready to stop the robot immediately if the arm, hand, or DEX3 open/close direction behaves unexpectedly.

The arm movement is intentionally small, but the correct sign and safe range still must be validated on the real robot.

## Requirements

- Ubuntu 20.04 or compatible Linux environment
- CMake 3.10 or newer
- GCC / G++ with C++17 support
- Unitree SDK2 headers, libraries, IDL files, and bundled DDS thirdparty files included in this repository
- Network connection to the R1 EDU DDS interface

## Build

```bash
cmake -B build
cmake --build build -j$(nproc)
```

## Run

Run an example with the right hand:

```bash
./build/bin/r1_arm_dex3_grasp_example <network_interface> R
```

Run an example with the left hand:

```bash
./build/bin/r1_arm_dex3_grasp_example <network_interface> L
```

Example:

```bash
./build/bin/r1_arm_dex3_grasp_example enp0s31f6 R
```

Find your network interface with:

```bash
ip addr
ip link
```

## DEX3 Hand Poses

The arm and hand examples use calibrated DEX3 motor pose arrays for the left and right hands. If open and close are reversed or need fine tuning on your hardware, adjust the open/closed pose arrays in the matching file under `example/r1/low_level/`.

## Repository Structure

```text
cmake/        CMake package files for SDK2 installation/use
example/r1/   Focused R1 + DEX3-1 manipulation example
include/      Unitree SDK2 headers and generated IDL message headers
lib/          Unitree SDK2 static libraries for supported architectures
licenses/     Preserved thirdparty and SDK license files
thirdparty/   Bundled DDS headers and shared libraries required by SDK2
```

## Development Roadmap

- Safer arm pose library
- Reusable R1 arm controller class
- Reusable DEX3 hand controller class
- Grasp presets
- Keyboard/manual control
- Trajectory logging
- Perception-based grasping
