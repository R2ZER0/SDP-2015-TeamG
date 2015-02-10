# motor_calibration.py
import numpy as np

# Raw data from experiments
_RAW_Y = np.array([100., 90., 80., 70., 60., 50., 40., 30., 20., 10.,  0.])[::-1]
_RAW_X = np.array([6.7 , 6.5, 6.4, 6.2, 6.0, 5.7, 5.3, 4.8, 3.8, 2.0, 0.0])[::-1]

def _normalise_array(data):
    """Nice 'n' simple - normalise an array"""
    norm_data = np.array(data, copy=True)
    norm_data /= np.max(np.abs(norm_data), axis=0)
    return norm_data

_NORMALISED_X = _normalise_array(_RAW_X)
_NORMALISED_Y = _normalise_array(_RAW_Y)

def get_calibrated_speed(speed):
    """Pass a speed -1 to 1, returns calibrated speed"""
    return np.sign(speed) * np.interp([np.abs(speed)], _NORMALISED_X, _NORMALISED_Y)[0]