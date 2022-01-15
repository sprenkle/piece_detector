#!/usr/bin/env python3  

from interbotix_xs_modules.arm import InterbotixManipulatorXS
import time

bot = InterbotixManipulatorXS("rx200", "arm", "gripper", gripper_pressure=1)
#bot.dxl.robot_set_motor_registers("single", "gripper", "Position_P_Gain", 1500)
#bot.dxl.robot_torque_enable("group", "arm", True)

print(f'self.left_finger_upper_limit) {bot.gripper.left_finger_upper_limit}')
print(f'self.left_finger_lower_limit) {bot.gripper.left_finger_lower_limit}')

bot.gripper.open()
time.sleep(2)

bot.gripper.close()
time.sleep(2)

bot.gripper.open()
time.sleep(2)

bot.gripper.close()
time.sleep(2)
