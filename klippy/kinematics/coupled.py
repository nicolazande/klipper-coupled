# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from . import idex_modes

class CoupledKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        printer_config = config.getsection('printer')
        ka = printer_config.getfloat('ka', 1.0, minval=-100., maxval=100.)
        kb = printer_config.getfloat('kb', 1.0, minval=-100., maxval=100.)
        # Setup axis rails
        rail_alpha = stepper.LookupMultiRail(config.getsection('stepper_x'))
        self.rails = [rail_alpha]
        rail_alpha.setup_itersolve('coupled_stepper_alloc', ka, kb)
        alpha_range = rail_alpha.get_range()
        self.axes_min = toolhead.Coord(alpha_range[0], 0., 0., alpha_range[1])
        self.axes_max = toolhead.Coord(alpha_range[1], 0., 0., alpha_range[0])
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off", self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.limits = [(1.0, -1.0)] * 3

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def calc_position(self, stepper_positions):
        return [stepper_positions[0]]
    
    def update_limits(self, i, range):
        l, h = self.limits[i]
        if l <= h:
            self.limits[i] = range

    def override_rail(self, i, rail):
        self.rails[i] = rail

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)

    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        # Each axis is homed independently and in order
        homing_state.set_axes([0])
        for axis in homing_state.get_axes():
            self.home_axis(homing_state, axis, self.rails[axis])

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
        
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CoupledKinematics(toolhead, config)
