#!/usr/bin/env python3  

from interbotix_xs_modules.arm import InterbotixManipulatorXS

import numpy as np
import time


bot = InterbotixManipulatorXS("rx200", "arm", "gripper")
bot.arm.go_to_home_pose()
bot.arm.go_to_sleep_pose()
