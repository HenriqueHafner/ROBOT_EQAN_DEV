# """
# Created on 2025
# @author: Henrique Hafner Ferreira
# @author: Henrique Itao Palos
# @company: OuroNova Brazil
# A planner for a positions aproximation, based in a aproximation function.
# Encapsulates goal setting, trajectory scaling, and parameter constraints for robotic motion control.
# This module is part of the Tankbotics motion control stack.
# """

from motion_control import trajectory_jerk_aproximation

class position_aproximation_planner:
    def __init__(self, id):
        self.id = int(id)
        self.params_defined = False
        #Manuever attributes
        self.displacement   = None
        self.start_position = None
        self.end_position   = None
        self.manuever_time  = None
        self.iteration_period = 1/60
        self.quantity_time_steps = None
        self.positions = []
        # Manuever speed limits
        self.max_jerk = None
        self.jerk_proportion = None
        self.min_i2_period = None
        self.min_displacement = None
        self.max_mean_velocity = None

    def set_params(self, max_velocity, max_acceleration, iteration_period):
        self.iteration_period = iteration_period
        if max_velocity <= 0 or max_acceleration <= 0:
            raise ValueError("maximum velocity and acelleration are not positive.")
        self.max_jerk = (max_acceleration ** 2) / max_velocity
        self.min_i2_period = 2 * max_velocity / max_acceleration
        self.min_displacement =  (5 / 3) * (max_velocity ** 2) / max_acceleration
        self.max_mean_velocity = (5 / 6) * max_velocity
        self.params_defined = True

    # max accelerations not checked yet, pending to implemenet
    def define_positions(self, start_posisiton, end_position, manuever_time):
        if not self.params_defined:
            print(f"w: ID:{self.id} Impossible to compute goals, params are not set.")
            return False
        displacement = end_position-start_posisiton 
        mean_velocity = displacement / manuever_time
        if mean_velocity > self.max_mean_velocity:
            warning = "w: ID:" + str(self.id) + " proposed goal is is above acceleration limit, adapting to max_mean_velocity: " + str(mean_velocity)
            print(warning)
            manuever_time = displacement/self.max_mean_velocity
        self.start_position = start_posisiton
        self.end_position = end_position
        self.displacement = displacement
        self.manuever_time = manuever_time
        positions = self.set_manuever_positions()
        if positions == [] or positions == None:
            print(f"w: ID:{self.id} Failed to compute valid trajectory positions.")
            return [None]
        return positions

    def set_manuever_positions(self):
        positions = trajectory_jerk_aproximation.get_manuever_positions(
            self.start_position, self.end_position,  self.manuever_time, self.iteration_period )
        self.positions = positions
        return positions
