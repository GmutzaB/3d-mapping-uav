"""Offline rigid-body quad model. SI units, world NED, body FRD; no hardware I/O."""
from dataclasses import dataclass, field
import argparse
import json
from pathlib import Path
import numpy as np

from hardware import MASS_KG, propeller


def _empirical_motor_ceiling():
    """Full-throttle output of the unvalidated static-thrust formula. Not a dyno curve."""
    return float(propeller(1.0)["thrust_per_motor_N"])


@dataclass(frozen=True)
class Parameters:
    mass: float = MASS_KG
    arm: float = 0.20  # CG-to-motor distance, unmeasured; motor spacing still clears a 9 inch prop
    inertia: tuple = (0.025, 0.025, 0.045)  # diagonal kg m^2, unmeasured
    max_thrust: float = field(default_factory=_empirical_motor_ceiling)
    yaw_ratio: float = 0.015  # reaction torque / thrust, m, unmeasured
    motor_tau: float = 0.04  # seconds, thrust response approximation
    drag: float = 0.0  # world isotropic linear drag N/(m/s)

    def __post_init__(self):
        positive = [self.mass, self.arm, self.max_thrust, self.motor_tau, *self.inertia]
        if len(self.inertia) != 3 or not np.all(np.isfinite(positive)) or min(positive) <= 0:
            raise ValueError('Mass, geometry, inertia, thrust and time constant must be positive and finite')
        if not np.all(np.isfinite([self.yaw_ratio, self.drag])) or min(self.yaw_ratio, self.drag) < 0:
            raise ValueError('Yaw ratio and drag must be finite and nonnegative')
        if max(self.inertia) > sum(self.inertia) - max(self.inertia):
            raise ValueError('Principal moments must satisfy the triangle inequality')


def rotation(q):
    """Body FRD to world NED; scalar-first unit quaternion."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('Quaternion must be finite and nonzero')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Quad:
    # State: position NED, velocity NED, q(wxyz), angular velocity FRD, motor thrusts.
    # Internal order: front-right, rear-right, rear-left, front-left; NOT FC numbering.
    def __init__(self, parameters=None):
        self.p = parameters or Parameters()
        a = self.p.arm / np.sqrt(2)
        self.positions = np.array([[a,a,0],[-a,a,0],[-a,-a,0],[a,-a,0]])
        self.yaw_sign = np.array([1,-1,1,-1])  # reaction torque sign about body +z
        self.state = np.zeros(17)
        self.state[6] = 1

    def hover_throttle(self):
        value = np.sqrt(self.p.mass * 9.81 / (4*self.p.max_thrust))
        if value > 1:
            raise ValueError('Assumed maximum thrust cannot support mass')
        return float(value)

    def wrench(self, thrust):
        forces = np.zeros((4,3))
        forces[:,2] = -np.asarray(thrust)
        torque = np.cross(self.positions, forces).sum(axis=0)
        torque[2] += self.p.yaw_ratio * np.dot(self.yaw_sign, thrust)
        return forces.sum(axis=0), torque

    def derivative(self, s, command, disturbance_torque=None):
        p = self.p
        q = s[6:10]
        omega = s[10:13]
        force, torque = self.wrench(s[13:17])
        if disturbance_torque is not None:
            torque = torque + disturbance_torque
        out = np.zeros(17)
        out[:3] = s[3:6]
        out[3:6] = rotation(q) @ force / p.mass + [0,0,9.81] - p.drag*s[3:6]/p.mass
        out[6] = -0.5*np.dot(q[1:], omega)
        out[7:10] = 0.5*(q[0]*omega + np.cross(q[1:], omega))
        inertia = np.asarray(p.inertia)
        out[10:13] = (torque - np.cross(omega, inertia*omega))/inertia
        out[13:17] = (p.max_thrust*command**2-s[13:17])/p.motor_tau
        return out

    def step(self, throttle, dt, disturbance_torque=(0., 0., 0.)):
        u = np.asarray(throttle, dtype=float)
        if u.shape != (4,) or not np.all(np.isfinite(u)) or np.any((u<0)|(u>1)):
            raise ValueError('Four finite throttle values in [0,1] required')
        if not np.isfinite(dt) or dt <= 0 or dt > self.p.motor_tau/2:
            raise ValueError('dt must be positive and <= motor_tau/2')
        if not np.all(np.isfinite(self.state)) or np.any(self.state[13:] < 0):
            raise ValueError('State must be finite with nonnegative thrust')
        disturbance_torque = np.asarray(disturbance_torque, dtype=float)
        if disturbance_torque.shape != (3,) or not np.all(np.isfinite(disturbance_torque)):
            raise ValueError('Disturbance torque must be a finite body-frame three-vector')
        s = self.state
        k1 = self.derivative(s,u,disturbance_torque)
        k2 = self.derivative(s+dt*k1/2,u,disturbance_torque)
        k3 = self.derivative(s+dt*k2/2,u,disturbance_torque)
        k4 = self.derivative(s+dt*k3,u,disturbance_torque)
        result = s+dt*(k1+2*k2+2*k3+k4)/6
        if not np.all(np.isfinite(result)):
            raise ValueError('Integration diverged')
        result[6:10] /= np.linalg.norm(result[6:10])
        self.state = result
        return result.copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['hover','freefall','roll'], default='hover')
    parser.add_argument('--seconds', type=float, default=None,
                        help='Duration (default: 0.5 s for roll, 2 s otherwise)')
    parser.add_argument('--dt', type=float, default=0.002)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'outputs/flight')
    parser.add_argument('--no-visuals', action='store_true', help='Skip PNG/GIF generation')
    args = parser.parse_args()
    quad = Quad()
    if args.seconds is None:
        args.seconds = 0.5 if args.scenario == 'roll' else 2.0
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('seconds must be positive and finite')
    if not np.isfinite(args.dt) or not 0 < args.dt <= quad.p.motor_tau/2:
        parser.error('dt must be positive and <= motor_tau/2')
    u = np.full(4,quad.hover_throttle())
    if args.scenario == 'freefall':
        u[:] = 0
    else:
        quad.state[13:] = quad.p.mass*9.81/4  # initialize at equilibrium
    if args.scenario == 'roll':
        u += np.array([-1,-1,1,1])*0.01
    rows = [np.r_[0,quad.state]]
    t = 0.0
    while t < args.seconds:
        dt = min(args.dt,args.seconds-t)
        state = quad.step(u,dt)
        t += dt
        rows.append(np.r_[t,state])
    args.output.mkdir(parents=True,exist_ok=True)
    np.savetxt(args.output / f'{args.scenario}.csv', rows, delimiter=',', comments='',
               header='time,north,east,down,vn,ve,vd,qw,qx,qy,qz,p,q,r,T_FR,T_RR,T_RL,T_FL')
    summary = dict(scenario=args.scenario, parameters=vars(quad.p),
                   assumed_hover_throttle=quad.hover_throttle(), final_state=quad.state.tolist(),
                   limitation='Uncalibrated offline model; no controller, contact, battery or sensor model.')
    (args.output/f'{args.scenario}.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    if not args.no_visuals:
        from flight_visuals import export_visuals
        for path in export_visuals(rows,quad,args.scenario,args.output):
            print(f'Visual saved: {path.resolve()}')


if __name__ == '__main__':
    main()
