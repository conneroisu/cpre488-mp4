# cpre488-mp4

MP-4: UAV Control

Big Items:
1.     Implementing Embedded Control
2.     PID Control Basics 
3.     Debugging over telemetry

## Crazyflie System Overview

![[Cascading-PID-Diagram.png]]

Figure 2. Crazyflie control diagram

The control process starts with the state estimator module receiving sensor data and using it to calculate the drone's current attitude (its rotation, i.e. roll, pitch, and yaw). The state estimator then sends the calculated attitude to the state controller module, which also receives a set-point from the commander module (in our case, this is user input specifying the desired attitude or attitude rate and thrust). The state controller module contains a cascading **PID** controller that uses the inputs from the state estimator to calculate the actuation force needed. That is then sent to the power distribution module where the actuation force is converted to motor power then the loop starts over again.

The Crazyflie runs off of a cascaded **PID** system where the output of the first **PID** controller is then used as an input for a second **PID** controller. This layout can be seen in figure 3, the output from the attitude **PID** controller, the desired attitude rate, becomes the input of the attitude rate **PID** controller. In part 2, we  implemented the attitude and attitude rate **PID** controllers for roll, pitch, and yaw. In part 1, we determined the **PID** values that should be present in the controllers by using a working **PID** controller before implementing it ourselves.

## Building Firmware

We found it useful to have a script for creating new configurations for building and flashing firmware to different drones as it kept our iteration times as short as possible.

We mounted our git repository onto the root of the virutal-machine by clicking `[Tab Item]` Devices -> Shared Folders, then adding a new folder that we would `cd` into each time we wanted to make firmware changes. Then we would just execute the `vm.sh` script to update the files in the part 2 software folder with their newest replacements. 

Reload in VM bash script (Copies git maintained files from a mounted git repo in the VM to the correct places to build firmware and generates a correct config.mk files for the specified drone)

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

Sometimes Windows screws up the line endings of the vm.sh file so this script can be used to fix it:

```bash
sed -i 's/\r$//' vm.sh
```
