#!/usr/bin/env python3

import os
import sys
from time import sleep

# This is a script that will allow you to move each indidual servo!
# With this you will be able to make custom movements.
# below is an example of how might you use this, build on it if you think it is helpful
# What you need to change is the pwm# value which matches each servo
# pwm1 = J15 connection on the board
# pwm 2 = J14...... pwm16 = J1
# ranges:  0 degree(echo 500000), 90 degree(echo 1500000), 180 degree(echo 2500000)

# The following test was done plugging in an extra servo to J15

zero = 500000
ninety = 1500000
one_eighty = 2500000 
total_degrees = 180
mid_degrees = 90


pwm_per_degree = int((one_eighty - zero) / total_degrees)


def move_servo(degree,output):
    global zero
    degree = round(zero + (pwm_per_degree * degree))
    os.system("echo " + str(degree) + " > /sys/class/pwm/pwmchip0/" + str(output) + "/duty_cycle")


def main():
    os.system("sudo systemctl stop robot")

    #Servos J2 and J3 are FR hip and calf respectively
    # This corresponds to pwm15 and pwm14

    port1 = "pwm14"
    pose1 = 0
    port2 = "pwm13"
    pose2 = 0

    while True:

        move_servo(pose1,port1)
        move_servo(pose2,port2)

        pose1 = int(input("Servo 1 Desired Angle: "))
        pose2 = int(input("Servo 2 Desired Angle: "))

        sleep(1)
    
    os.system("sudo systemctl stop robot")



if __name__ == "__main__":
    main()