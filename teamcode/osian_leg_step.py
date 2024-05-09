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


pwm_per_degree = (one_eighty - zero) / total_degrees


def move_servo(degree,output):
    global zero
    degree = round(zero + (pwm_per_degree * degree))

    os.system("echo " + str(degree) + " > /sys/class/pwm/pwmchip0/" + str(output) + "/duty_cycle")

    print("done")

def main():
    os.system("sudo systemctl stop robot")

    #Servos J2 and J3 are FR hip and calf respectively
    # This corresponds to pwm15 and pwm14

    thigh = "pwm14"
    calf = "pwm13"
    thigh_initial = 30
    calf_initial = 25
    
    #Create a list of positions:
    steps = [0,1,2,3,4]
    thigh_steps = [45,80,80,60,45,30]
    calf_steps = [5,0,15,15,20,25]
    
    #initialize our servos:
    move_servo(thigh_initial,thigh)
    move_servo(calf_initial,calf)
    sleep(1)
    
    for j in steps:
        for i in steps:
            print(i)
            move_servo(thigh_steps[i], thigh)
            move_servo(calf_steps[i], calf)
            sleep(0.1)

    os.system("sudo systemctl stop robot") 



if __name__ == "__main__":
    main()
