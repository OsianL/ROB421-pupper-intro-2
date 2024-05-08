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
one_eight = 2500000 
total_degrees = 180
mid_degrees = 90

total_pwm_change_first_half = ninety - zero
total_pwm_change_second_half = one_eight - zero


pwm_per_degree_first_half = total_pwm_change_first_half/mid_degrees
pwm_per_degree_second_half = total_pwm_change_second_half/total_degrees


def move_servo(degree,output):
    global zero
    degree
    degree_finder = zero + (pwm_per_degree_first_half * degree)


    os.system("echo " + str(degree_finder) + " > /sys/class/pwm/pwmchip0/" + str(output) + "/duty_cycle")
    
    #time.sleep(1)
    #os.system("echo 2500000 > /sys/class/pwm/pwmchip0/pwm1/duty_cycle")
    #time.sleep(1)
    #os.system("echo 500000 > /sys/class/pwm/pwmchip0/pwm1/duty_cycle")

    print("done")

def main():
    os.system("sudo systemctl stop robot")
    thigh = "pwm9"
    calf = "pwm10"
    thigh_initial = 90
    calf_initial = 90
    #Create a list of positions:
    steps = [0,1,2,3,4]
    thigh_steps = [30,45,30,15,0]
    calf_steps = [30,-14,-5,-2,0]
    move_servo(thigh_initial,thigh)
    move_servo(calf_initial,calf)
    sleep(1)
    for i in steps:
        move_servo(thigh_steps(i) + thigh_initial, thigh)
        move_servo(calf_steps(i) + calf_initial, calf)
        sleep(1)



if __name__ == "__main__":
    main()