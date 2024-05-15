#
# Copyright 2024 MangDang (www.mangdang.net) 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description: AI camera samples based on K210 module.
# Attention: 
#   This is ONLY the simple sample code based on K210 module for your reference.
#   We tested some other functions in the module but the performance is not as good as expected.
#   We DON'T recommend you order the camera module and we have no relationship with the camera vendor.
#
# Before test:
#   Make sure the software is right inside camera module, and please refer to the guide.
#   Prepare the map with the line.
# Test method: 
#   step1: run "python AICameraSamples.py"
#   step2: Input mode:
#          0: line following function
#          1: face following function
#   step3: CTL+c to stop the function 
#
#   To Do List:
#   If you want to use some signal(speech or gesture) to start/stop the function, you can customize this sample code.
#
#   Reference: https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/master/run_robot.py
#

from sshkeyboard import listen_keyboard
import asyncio
import numpy as np
import time
from src.Controller import Controller
from src.Command import Command
from src.State import BehaviorState, State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display


keypressed = ''

async def press(key):
    global keypressed
    print("\n " + key)
    keypressed = key
    

def main():
    """Main program
    """

    global keypressed
    
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create controller
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    #Setup State and command instances
    state = State()
    state.quat_orientation = np.array([1,0,0,0])
 
    command = Command()

    #Handle the first loop iteration
    firstLoopFlag = True
    last_loop = time.time()
    
    listen_keyboard(on_press=press)

    while True:
        now = time.time()
        if now - last_loop < config.dt:
            continue

        if(firstLoopFlag):
            firstLoopFlag = False
            state.behavior_state = BehaviorState.REST
        else:
            state.behavior_state = BehaviorState.TROT
            
        last_loop = time.time()
        
        #adjust velocity based on keyboard presses:
        if(keypressed == 'w'):
            command.horizontal_velocity = np.array([0.2,0])
        elif(keypressed == 's'):
            command.horizontal_velocity = np.array([-0.2,0])
        elif(keypressed == 'd'):
            command.horizontal_velocity = np.array([0,0.2])
        elif(keypressed == 'a'):
            command.horizontal_velocity = np.array([0,-0.2])
        elif(keypressed == None):
            command.horizontal_velocity = np.array([0,0])
        elif(keypressed == 'r'):
            command.horizontal_velocity = np.array([0,0])
            state.behavior_state = BehaviorState.DEACTIVATED
            break

		
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)
        
main()
