"""Explicit ROS ENU/FLU to aircraft NED/FRD basis conversions."""
import numpy as np
from flight_dynamics import rotation

ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_TO_FRD = np.diag([1., -1., -1.])


def vector(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError('Expected a finite three-vector')
    return value


def enu_to_ned(value):
    return ENU_TO_NED @ vector(value)


def flu_to_frd(value):
    return FLU_TO_FRD @ vector(value)


def attitude_enu_flu_to_ned_frd(quaternion_wxyz):
    """Return rotation matrix FRD->NED, NOT just a world-only basis change."""
    return ENU_TO_NED @ rotation(quaternion_wxyz) @ FLU_TO_FRD


def covariance_enu_to_ned(value):
    """Transform a 3x3 vector covariance; angles need their own error convention."""
    value = np.asarray(value, dtype=float)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ValueError('Expected a finite 3x3 covariance')
    if not np.allclose(value, value.T, atol=1e-12, rtol=0) or np.linalg.eigvalsh(value).min() < -1e-12:
        raise ValueError('Covariance must be symmetric positive semidefinite')
    return ENU_TO_NED @ value @ ENU_TO_NED.T
