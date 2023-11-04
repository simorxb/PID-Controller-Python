import numpy as np

class PID:

    """ This class implements a PID controller.
    """

    def __init__(self, Kp, Ki, Kd, Kaw, T_C, T, max, min, max_rate):
        self.Kp = Kp                # Proportional gain
        self.Ki = Ki                # Integral gain
        self.Kd = Kd                # Derivative gain
        self.Kaw = Kaw              # Anti-windup gain
        self.T_C = T_C              # Time constant for derivative filtering
        self.T = T                  # Time step
        self.max = max              # Maximum command
        self.min = min              # Minimum command
        self.max_rate = max_rate    # Maximum rate of change of the command
        self.integral = 0           # Integral term
        self.err_prev = 0           # Previous error
        self.deriv_prev = 0         # Previous derivative
        self.command_sat_prev = 0   # Previous saturated command
        self.command_prev = 0       # Previous command
        self.command_sat = 0        # Current saturated command
        self.command = 0            # Current command

    def Step(self, measurement, setpoint):
        """ Execute a step of the PID controller.

        Inputs:
            measurement: current measurement of the process variable
            setpoint: desired value of the process variable
        """

        # Calculate error
        err = setpoint - measurement

        # Update integral term with anti-windup
        self.integral += self.Ki*err*self.T + self.Kaw*(self.command_sat_prev - self.command_prev)*self.T
        
        # Calculate filtered derivative
        deriv_filt = (err - self.err_prev + self.T_C*self.deriv_prev)/(self.T + self.T_C)
        self.err_prev = err
        self.deriv_prev = deriv_filt

        # Calculate command using PID equation
        self.command = self.Kp*err + self.integral + self.Kd*deriv_filt

        # Store previous command
        self.command_prev = self.command

        # Saturate command
        if self.command > self.max:
            self.command_sat = self.max
        elif self.command < self.min:
            self.command_sat = self.min
        else:
            self.command_sat = self.command

        # Apply rate limiter
        if self.command_sat > self.command_sat_prev + self.max_rate*self.T:
            self.command_sat = self.command_sat_prev + self.max_rate*self.T
        elif self.command_sat < self.command_sat_prev - self.max_rate*self.T:
            self.command_sat = self.command_sat_prev - self.max_rate*self.T

        # Store previous saturated command
        self.command_sat_prev = self.command_sat

class Car:

    """ This class represents a car moving in 1D, subject to a throttle force F, with mass m, 
        aerodynamic drag coefficient b, F_max/F_min forces, and time step T. 
    """

    def __init__(self, m, b, F_max_0, F_max_max, v_max, T):
        self.m = m                      # Mass of the car
        self.b = b                      # Aerodynamic drag coefficient
        self.F_max_0 = F_max_0          # Max force applied to the car by the powertrain at 0 speed
        self.F_max_max = F_max_max      # Max force applied to the car by the powertrain at max speed
        self.v_max = v_max              # Max speed (m/s)
        self.T = T                      # Time step
        self.v = 0                      # Speed of the car

    def Step(self, F):

        """ Update the speed of the car based on the applied force F.
        """
        # Max force applied by the powertrain depends on the speed
        v_to_F_max_x_axis = [0, self.v_max]
        F_max_y_axis = [self.F_max_0, self.F_max_max]

        if self.v < v_to_F_max_x_axis[0]:
            F_max = F_max_y_axis[0]
        elif self.v > v_to_F_max_x_axis[-1]:
            F_max = F_max_y_axis[-1]
        else:
            F_max = np.interp(self.v, v_to_F_max_x_axis, F_max_y_axis)

        # Saturate input force
        if F > F_max:
            F_sat = F_max

        elif F < 0:
            F_sat = 0
        else:
            F_sat = F

        # Calculate the derivative dv/dt using the input force and the car's speed and properties
        dv_dt = (F_sat - self.b*self.v*self.v)/self.m

        # Update the speed by integrating the derivative using the time step T
        self.v += dv_dt*self.T