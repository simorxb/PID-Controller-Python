import numpy as np
from lib import Car, PID
import matplotlib.pyplot as plt

def main():
    # -------- Configuration --------

    # Simulation parameters

    time_step = 0.1
    end_time = 25
    length = round(end_time/time_step)

    t = np.zeros(length)
    stp = np.zeros(length)
    v = np.zeros(length)
    command = np.zeros(length)

    # Car parameters

    m = 2140
    b = 0.33
    F_max_0 = 22000
    F_max_max = 1710
    v_max = 72

    # PID parameters

    Kp = 800.0
    Ki = 70.0
    Kaw = 1.0
    Kd = 20.0
    T_C = 1.0

    # Initialize PID controller
    pid = PID(Kp, Ki, Kd, Kaw, T_C, time_step, F_max_0, 0, 30000)

    # Initialize car with given parameters
    car = Car(m, b, F_max_0, F_max_max, v_max, time_step)

    # Iterate through time steps
    for idx in range(0, length):
        t[idx] = idx*time_step
        # Set setpoint
        stp[idx] = 42
        
        # Execute the control loop
        v[idx] = car.v
        pid.Step(v[idx], stp[idx])
        command[idx] = pid.command_sat
        car.Step(command[idx])  

    # Plot speed response

    plt.subplot(2, 1, 1)
    plt.plot(t, v, label="Response")
    plt.plot(t, stp, '--', label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.legend()
    plt.grid()

    # Plot command force

    plt.subplot(2, 1, 2)
    plt.plot(t, command, label="Command")
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.legend()
    plt.grid()

    # Display the plots

    plt.show()

main()