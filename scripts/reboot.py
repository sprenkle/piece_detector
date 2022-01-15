#!/usr/bin/env python3  

from interbotix_xs_modules.arm import InterbotixManipulatorXS

bot = InterbotixManipulatorXS("rx200", "arm", "gripper", gripper_pressure=1)
bot.core.robot_reboot_motors("group", "arm", False)