import numpy
from typing import Sized
from math import radians
import build_functions


class RawAttr:
    motor_range={1: [1.0, 6.35], 2: [0.96, 1.78], 3: [0.85, 1.325], 4: [-125.0, 125.0], 5: [-99999999.0, 999999999.0],
                  6: [-80.0, 90.0]}
    motor_vel={1: 0.3, 2: 0.2, 3: 0.1, 4: 35.0, 5: 180.0, 6: 60.0}
    motor_acc={1: 0.5, 2: 0.3, 3: 0.2, 4: 60.0, 5: 360.0, 6: 180.0}
    shoot_range_xy= 0.25
    l1_length= 0.5629
    line_speed=0.53
    command_interval= 0.02
    dis_l1l1= 0.1
    raw_xy:list


class Builder:
    '''
    command:        [ts_sum, m1_tar, m2, m3, m4, m5, m6]
    commands:       [<command1>, <command2>, <command3>, ...]
    bots_commands:  {1: <commands1>, 2:<commands2>}
    '''
    bots_commands: dict
    def __init__(self, car:list):
        self.raw_attr =RawAttr()
        self.raw_attr.raw_xy = car
        self.bots_commands = {1: [], 2: []}

