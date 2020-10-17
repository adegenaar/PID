[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![PythonTemplate](https://github.com/adegenaar/PythonTemplate/workflows/PythonTemplate/badge.svg)

# PID Controller for Python
**Simple PID control class for Python.**

based on the C example found here: [PID Controller Written in C](https://github.com/pms67/PID)

More details on PID loop control can be found on [Wikipedia](https://en.wikipedia.org/wiki/PID_controller)

A calculator for tuning the PID loop is available [here](https://pidtuner.com/#/)

Usage is simple:
* Create the a PID object
    * pid = PIDController() 
    * pid = PIDController(kp, ki, kd)
* Set the initial values:
    * kp - (Proportional)
    * ki - (Integral)
    * kd - (Derivative)
    * limmin - Lower limit on the output delta 
    * limmax - Upper Limit on the output delta
    * tau - Lag term for the Integrator
    * T - Time is seconds
    * limMinInt - Lower limit for the Integrator
    * limMaxInt - Upper limit for the Integrator
* Loop, updating the expected (setpoint) and actual (measurement) values for a given time
    ```
    setpoint = 1.0

    print("Time(s)\tSystemOut\tController Out")
    t = 0.0
    while t < 4.0:
        measurement = <function for real values>(pid.out)
        pid.update(setpoint, measurement)
        print(f"{t}\t{measurement}\t{pid.out}")
        t += 0.01
    ```

    