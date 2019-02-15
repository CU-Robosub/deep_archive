# Pololu_Motor
This package contains packages for interfacing with the pololu and teleop control of the sub.

This repo is designed to be placed in a catkin workspace. [Our Workspace](https://github.com/CU-Robosub/Final_Build)
## Contents
* pololu_controller - Package for controlling the [Pololu Maestro](https://www.pololu.com/product/1352) in ROS.
* teleop - Teleop scripts for testing
## Usage
### Pololu Controller
* Listens to ros topic /pololu/command for pwn position.
* Motor names are defined in the yaml
## Teleop
[command, intensity] ie... a4, s, r6

pulse_width = intensity*10 + 1500

keyboard[command] locations
mirror the sub layout

| Position | Left | Center | Right |
| -------- |:----:|:------:|:-----:|
| Front    | q | w | e |
| Center   | a |   | d |
| Back     | z | x | c |

### s kills all motors

## Pololu Pins
* Front: 0
* front_right: 6
* front_left: 2
* back_right: 3
* back_left: 5
* back: 4
* left: 9
* right: 8
