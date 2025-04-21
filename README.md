# cpre488-mp4
CprE 488 \- Embedded Systems Design

MP-4: UAV Control (ver. 2.0)

**Assigned**: Monday of Week 10 **Due**: Monday of Week 12 **Points**: 100 + bonus points

*Note: the goal of this Machine Problem is for you to work with your group to increase your exposure to three different aspects of embedded system design:*

1.     Implementing Embedded Control — you will learn to implement control theory into a discrete system capable of running on a microcontroller.

2.     Understanding PID Control Basics — you will tune PID loops to gain familiarity with the impact of different values.

3.     Debugging over telemetry — you will learn to debug systems using parameter logging over wireless communication.

## Your Mission

[A MAN HAS FALLEN INTO THE RIVER IN LEGO CITY!](https://www.youtube.com/watch?v=Rl1HcsdVf0w) Start the new rescue quadcopter! HEY! Program and tune the quadcopter, and off to the rescue! Prepare the lifeline, lower the stretcher, and make the rescue! The Crazyflie collection from MicroCART!

Say hello to your Crazyflie drone! The goal at the end of this lab is to be able to smoothly control the Crazyflie with the algorithms that you will write and test. First, we will give you a brief overview of the Crazyflie system and how to set up the development environment for this lab.

This lab has been developed over the course of several years by the MicroCart Senior Design project. The [MicroCart Youtube Channel](https://www.youtube.com/%40microcart8754) has many videos you may find helpful in case you get stuck during this lab, or for ideas you can do for extra credit.

  

## Part 0: Meet the Crazyflie.

There is only one physical button on the Crazyflie, the power button. To start the Crazyflie:

1.     Plug in a charged battery

2.     ![](https://class.ece.iastate.edu/cpre488/labs/MP4/MP-4_files/image002.jpg)Press the power button located near the front right arm of the drone

3.     Wait for all four props to do a short spin and the startup tone to play.

4.     Place the Crazyflie on a flat surface to allow its sensors to calibrate. This is indicated by the flashing red LED. If it flashes quickly, then the sensors have been calibrated properly and the drone is ready to fly. If the LED flashes slowly, then the sensors have not been calibrated yet. If some hardware is damaged on the Crazyflie it may fail to pass its self check on startup. This is indicated by the red led flashing quickly 5 times. In this case the hardware may be inoperable. If this is the case, notify a TA or the instructor.

##### 5.     Always secure the CrazyFlie to a test-stand during PID tuning, and Controller development

Figure 1. Crazyflie top down diagram

| **LED Lights** | **Diagnostic** |
| --- | --- |
| flashing red LED | Calibrating |
| Slow flashing LED | Sensors have not been calibrated |
| Red LED flashing 5 times quickly | Failed to pass self checkup. Talk to TA if this happens |

## Crazyflie System Overview

This is the complete Crazyflie control system. You will only be modifying a small portion of it, but it will be helpful to understand the full scope of the system you are interfacing with.

  

![](https://class.ece.iastate.edu/cpre488/labs/MP4/MP-4_files/image004.jpg)

Figure 2. Crazyflie control diagram

The control process starts with the state estimator module receiving sensor data and using it to calculate the drone's current attitude (its rotation, i.e. roll, pitch, and yaw). The state estimator then sends the calculated attitude to the state controller module, which also receives a setpoint from the commander module (in our case, this is user input specifying the desired attitude or attitude rate and thrust). The state controller module contains a cascading PID controller that uses the inputs from the state estimator to calculate the actuation force needed. That is then sent to the power distribution module where the actuation force is converted to motor power then the loop starts over again.

You will be implementing the State Controller’s cascading PID in part 2 of this lab.

|  |
| --- |
|  | ![](https://class.ece.iastate.edu/cpre488/labs/MP4/MP-4_files/image006.jpg) |

  

Figure 3. Cascading PID diagram

The Crazyflie runs off of a cascaded PID system where the output of the first PID controller is then used as an input for a second PID controller. This layout can be seen in figure 3, the output from the attitude PID controller, the desired attitude rate, becomes the input of the attitude rate PID controller. In part 2, we  implemented the attitude and attitude rate PID controllers for roll, pitch, and yaw. In part 1, you determined the PID values that should be present in the controllers by using a working PID controller before implementing it yourself.


Reload in Vm bash script (Copies git maintained files from a mounted git repo in the vm to the correct places to build firmware and generates a correct config.mk files for the specified drone)
```bash
#!/usr/bin/bash
# Function to display usage information
show_usage() {
    echo "Usage: $0 [-n CRAZYFLIE_NUMBER]"
    echo "If -n is not provided, you will be prompted for the CrazyFlie number."
}

# Function to get channel number from CrazyFlie number
get_channel() {
    local cf_number=$1
    
    case $cf_number in
        4)  echo "40" ;;
        5)  echo "50" ;;
        7)  echo "70" ;;
        8)  echo "80" ;;
        11) echo "100" ;;
        12) echo "95" ;;
        16) echo "85" ;;
        17) echo "65" ;;
        0)  echo "30" ;;  # For unlabeled
        *)  echo "Invalid CrazyFlie number" >&2; exit 1 ;;
    esac
}

# Parse command line arguments
cf_number=""
while getopts "n:h" opt; do
    case $opt in
        n) cf_number=$OPTARG ;;
        h) show_usage; exit 0 ;;
        *) show_usage; exit 1 ;;
    esac
done

# If CrazyFlie number not provided, prompt for it
if [ -z "$cf_number" ]; then
    echo "Please enter your CrazyFlie number (use 0 for unlabeled):"
    read cf_number
fi

# Get channel number
channel=$(get_channel $cf_number)

# Check if channel is valid
if [[ $channel == "Invalid CrazyFlie number" ]]; then
    echo "Error: Invalid CrazyFlie number: $cf_number"
    exit 1
fi

# Existing script content
echo "controller_student copying..."
cp ./src/controller_student.c          ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/src/controller_student.c
echo "controller_student copied..."
echo "student_attitude_controller.c copying..."
cp ./src/student_attitude_controller.c ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/src/student_attitude_controller.c
echo "student_attitude_controller.c copied"
echo "student_pid.c copying..."
cp ./src/student_pid.c                 ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/src/student_pid.c
echo "student_pid.c copied"

echo "student_attitude_controller.h copying..."
cp ./src/student_attitude_controller.h ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/interface/student_attitude_controller.h
echo "student_attitude_controller.h copied"
echo "student_pid.h copying..."
cp ./src/student_pid.h ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/interface/student_pid.h
echo "student_pid.h copied"

# Config file generation
echo "Generating config file for CrazyFlie #$cf_number (Channel $channel)..."
echo "CLOAD_CMDS=-w radio://0/$channel/2M/E7E7E7E7E7" > ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/tools/make/config.mk
echo "Config file generated successfully."

code ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/
```