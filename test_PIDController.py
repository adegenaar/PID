import pytest
from PIDController import PIDController


def test_new_controller():
    pid = PIDController()
    assert (pid is not None)


def test_new_controller_init():
    pid = PIDController()
    assert (pid is not None)
    assert (pid.Kp == 0.0)
    assert (pid.Kd == 0.0)
    assert (pid.Ki == 0.0)


def test_new_controller_update():
    pid = PIDController(kp=2.0, ki=0.5, kd=0.25, limmin=-10.0, limmax=10.0)
    assert (pid is not None)

    pid.tau = 0.02
    pid.limMinInt = -5.0
    pid.limMaxInt = 5.0
    pid.T = 0.01

    setpoint = 1.0

    print("Time(s)\tSystemOut\tController Out")
    t = 0.0
    while t < 4.0:
        measurement = _SimulatorUpdate(pid.out)
        pid.update(setpoint, measurement)
        print(f"{t}\t{measurement}\t{pid.out}")
        t += 0.01


def _SimulatorUpdate(inp):
    alpha = 0.02
    output = 0.0
    output = (0.01 * inp + output) / (1.0 + alpha * 0.01)
    return output
