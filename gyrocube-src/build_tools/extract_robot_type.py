# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of gyrocube-src.
#
# gyrocube-src is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# gyrocube-src is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with gyrocube-src.  If not, see <https://www.gnu.org/licenses/>.

from SCons.Script import *

ROBOT_TYPE_FILE     = "robot-type/robot_type.hpp"
VALID_ROBOT_TYPES   = [ "TARGET_GYROCUBE" ]

def get_robot_type():
    robot_type = ARGUMENTS.get("robot")
    # Configure robot type and check against valid robot type
    # If there is no optional argument, revert back to the macro in robot_type.hpp
    if robot_type == None:
        with open(ROBOT_TYPE_FILE, "r") as robot_type_file_reader:
            for word in robot_type_file_reader.read().splitlines():
                if "#" in word and "define" in word and "TARGET_" in word:
                    robot_type = word.split()[-1]
                    break
    if robot_type not in VALID_ROBOT_TYPES:
        raise Exception(f"\n\n{robot_type} is not one of the valid robot types: {VALID_ROBOT_TYPES}\n\n")

    return robot_type
