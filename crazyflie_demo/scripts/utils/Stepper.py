"""Stepper is a tool for deviding route from point A to point B into N steps.

"""


class Stepper:
    """a Class for a Stepper object."""

    def __init__(self, n_steps_devider=10, logger=None):
        self.logger = logger
        self.n_steps = n_steps_devider
        self.current_point = None
        self.desired_point = None

    def update(self, current_point, target_point):
        """a function for updating Stepper object with target point. """
        self.current_point = current_point
        self.desired_point = target_point

#   TODO: add private function for calculating and appending all steps into an array and than fly the Crazyflie from this array, while listening and comparing Crazyflie location with desired point
#   TODO: add Stepper listener for calculating the distance between current step and next step.
