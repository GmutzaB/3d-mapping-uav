"""MAVLink 2 wire encoder; no socket, serial, arming or parameter writes here."""
import math
from pymavlink.dialects.v20 import ardupilotmega as mav
from .frames import vector


def uint(value, bits, name):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**bits:
        raise ValueError(f'{name} must be uint{bits}')
    return value


class Encoder:
    def __init__(self, source_system=191, source_component=191):
        for value in (source_system, source_component):
            if not 1 <= uint(value,8,'source ID') <= 255:
                raise ValueError('Source IDs must be nonzero')
        self.codec = mav.MAVLink(None, srcSystem=source_system, srcComponent=source_component)

    def packet(self, message):
        result = message.pack(self.codec)
        self.codec.seq = (self.codec.seq+1) % 256
        return result

    def velocity(self, velocity_ned, time_boot_ms, target_system, target_component=1):
        velocity = vector(velocity_ned)
        uint(time_boot_ms,32,'time_boot_ms')
        for value in (target_system, target_component):
            if not 1 <= uint(value,8,'target ID') <= 255:
                raise ValueError('Explicit nonzero target IDs required')
        mask = (mav.POSITION_TARGET_TYPEMASK_X_IGNORE | mav.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_Z_IGNORE | mav.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_AY_IGNORE | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_YAW_IGNORE | mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        return self.packet(mav.MAVLink_set_position_target_local_ned_message(
            time_boot_ms, target_system, target_component, mav.MAV_FRAME_LOCAL_NED,
            mask, 0,0,0, *velocity, 0,0,0, 0,0))

    def odometry(self, *, time_usec, position_local_frd, q_body_to_local_wxyz,
                 velocity_body_frd, angular_velocity_body_frd, pose_covariance,
                 reset_counter, quality):
        """Input is already calibrated/converted local FRD + body FRD (not raw ROS).

        Pose covariance is the MAVLink upper triangle of a 6x6 covariance. Full
        convention-aware covariance conversion belongs upstream. Unknown velocity
        covariance is encoded with NaNs, never fabricated certainty.
        """
        import numpy as np
        uint(time_usec,64,'time_usec')
        uint(reset_counter,8,'reset_counter')
        if isinstance(quality,bool) or not isinstance(quality,int) or not -1 <= quality <= 100:
            raise ValueError('Quality must be an integer in [-1,100]')
        q = np.asarray(q_body_to_local_wxyz,dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1) > 1e-5:
            raise ValueError('A finite unit quaternion is required')
        cov = np.asarray(pose_covariance,dtype=float)
        if cov.shape != (21,) or not np.all(np.isfinite(cov)):
            raise ValueError('Known finite pose covariance is required')
        matrix = np.zeros((6,6)); indices = np.triu_indices(6)
        matrix[indices] = cov
        matrix = matrix + np.triu(matrix,1).T
        if np.linalg.eigvalsh(matrix).min() < -1e-10:
            raise ValueError('Pose covariance must be positive semidefinite')
        return self.packet(mav.MAVLink_odometry_message(
            time_usec, mav.MAV_FRAME_LOCAL_FRD, mav.MAV_FRAME_BODY_FRD,
            *vector(position_local_frd), q.tolist(), *vector(velocity_body_frd),
            *vector(angular_velocity_body_frd), cov.tolist(), [math.nan]*21,
            reset_counter, mav.MAV_ESTIMATOR_TYPE_VISION, quality))
