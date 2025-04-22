# cpre488-mp4

MP-4: UAV Control

*Note: the goal of this Machine Problem is for you to work with your group to increase your exposure to three different aspects of embedded system design:*

1.     Implementing Embedded Control — you will learn to implement control theory into a discrete system capable of running on a microcontroller.

2.     Understanding PID Control Basics — you will tune PID loops to gain familiarity with the impact of different values.

3.     Debugging over telemetry — you will learn to debug systems using parameter logging over wireless communication.

Figure 1. Crazyflie top down diagram

| **LED Lights** | **Diagnostic** |
| flashing red LED | Calibrating |
| Slow flashing LED | Sensors have not been calibrated |
| Red LED flashing 5 times quickly | Failed to pass self checkup. Talk to TA if this happens |

## Crazyflie System Overview

![](https://class.ece.iastate.edu/cpre488/labs/MP4/MP-4_files/image004.jpg)

Figure 2. Crazyflie control diagram

The control process starts with the state estimator module receiving sensor data and using it to calculate the drone's current attitude (its rotation, i.e. roll, pitch, and yaw). The state estimator then sends the calculated attitude to the state controller module, which also receives a setpoint from the commander module (in our case, this is user input specifying the desired attitude or attitude rate and thrust). The state controller module contains a cascading PID controller that uses the inputs from the state estimator to calculate the actuation force needed. That is then sent to the power distribution module where the actuation force is converted to motor power then the loop starts over again.

You will be implementing the State Controller’s cascading PID in part 2 of this lab.

![](https://class.ece.iastate.edu/cpre488/labs/MP4/MP-4_files/image006.jpg)

**UAV Control Lab Report**

For this lab, our team worked collaboratively to implement and test key components of the UAV flight control system, focusing on developing a custom controller for attitude and thrust regulation. The primary goal of this lab was to write and tune our own control algorithms that would allow the drone to maintain stability and respond correctly to input commands.

**Implementation**

Our first step was to integrate the student controller code into the flight control software. We worked inside the `controller_student.c` and `student_attitude_controller.c` files. We created PID controllers for roll rate, pitch rate, and yaw rate. Each controller required initialization, error computation, and output saturation to keep control signals within safe bounds.

To ensure correct functioning, we added logging for each axis's PID outputs as well as the target attitude and rate commands. Being limited to twelve logging variables greatly limited our ability to debug. This data helped us monitor the controller's behavior in real time during flights.

**Tuning and Testing**

The most challenging part of the lab was tuning the PID gains. We started with conservative values and performed incremental tuning after each test flight. We observed oscillations in the pitch axis at first, which we mitigated by lowering the proportional gain and increasing the derivative term slightly.

We used the Crazyflie client interface to visualize the attitude angles and rate responses. During tests, we recorded flight data and analyzed logs using tools provided with the lab materials. This slightly helped us verify that our controller tracked the desired attitude and rate commands.

**Troubleshooting**

TODO : Add all the troubleshooting we did section
- Trash number of operable drones

Going forward, we feel more confident in working with PID controllers and embedded flight systems. This lab was a great foundation for more advanced UAV development tasks.

Figure 3. Cascading PID diagram

The Crazyflie runs off of a cascaded PID system where the output of the first PID controller is then used as an input for a second PID controller. This layout can be seen in figure 3, the output from the attitude PID controller, the desired attitude rate, becomes the input of the attitude rate PID controller. In part 2, we  implemented the attitude and attitude rate PID controllers for roll, pitch, and yaw. In part 1, you determined the PID values that should be present in the controllers by using a working PID controller before implementing it yourself.

## Building Firmware

We found it useful to have a script for creating new configurations for building and flashing firmware to different drones as it kept our iteration times as short as possible.

We mounted our git repo onto the root of the vm by clicking [Tab Item] Devices -> Shared Folders, then adding a new folder that we would cd into each time we wanted to make firmware changes. Then we would just execute the `vm.sh` script to update the files in the part 2 software folder with their newest replacements. 

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


Sometimes windows screws up the line endings of the vm.sh file so this script can be used to fix it:
```bash
sed -i 's/\r$//' vm.sh
```
