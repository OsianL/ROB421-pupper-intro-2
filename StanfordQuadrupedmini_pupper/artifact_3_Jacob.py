# pup2.engr.oregonstate.edu
# U: ubuntu P: mangdang

import numpy as np
import time
from src.Controller import Controller
from src.Command import Command
from src.State import BehaviorState, State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display

def main():
    """Main program
    """
    
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
    
    mode = input('input mode: 0 is Forward and 1 is horizontal: ')
    
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
        
        # EDIT HERE
        if(mode == '0'):
            command.horizontal_velocity = np.array([0.15,0])
        elif(mode == '1'):
            command.horizontal_velocity = np.array([0,0.15])
		
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)
        
main()