#!/bin/bash
# Running from the root of the cloned repo which should be reflected onto the vm in some folder run `sh vm.sh`.


cp ./src/controller_student.c ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/src/controller_student.c
cp ./src/student_attitude_controller.c ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/src/student_attitude_controller.c
cp ./src/student_pid.c ../MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/src/student_pid.c

cp ./src/student_attitude_controller.h ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/interface/student_attitude_controller.h
cp ./src/student_pid.h ~/MicroCART/crazyflie_software/crazyflie-firmware-lab-part-2/src/modules/interface/student_pid.h

