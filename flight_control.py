"""Simulation-only attitude/height feedback. No MAVLink or hardware interfaces."""
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse
import json
import numpy as np
from flight_dynamics import Quad, rotation


@dataclass(frozen=True)
class Gains:
    attitude_frequency: float = 6.0  # rad/s, provisional tuning
    damping: float = 0.9
    height_kp: float = 4.0
    height_kd: float = 3.5

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Gains must be positive and finite')


class Controller:
    def __init__(self, quad, gains=None):
        self.p = quad.p
        self.gains = gains or Gains()
        # [total upward thrust, body roll/pitch/yaw torque] = allocation @ rotor thrusts.
        self.allocation = np.column_stack([np.r_[1.,quad.wrench(np.eye(4)[i])[1]] for i in range(4)])
        self.inverse = np.linalg.inv(self.allocation)

    def command(self, state, roll_target=0.):
        state = np.asarray(state,dtype=float)
        if state.shape != (17,) or not np.all(np.isfinite(state)):
            raise ValueError('Expected a finite 17-state vector')
        if not np.isfinite(roll_target) or abs(roll_target) > np.radians(30):
            raise ValueError('Demo roll target must be within 30 degrees')
        R = rotation(state[6:10])
        c,s = np.cos(roll_target),np.sin(roll_target)
        desired = np.array([[1,0,0],[0,c,-s],[0,s,c]])
        skew = .5*(desired.T@R-R.T@desired)
        error = np.array([skew[2,1],skew[0,2],skew[1,0]])
        omega = state[10:13]
        g = self.gains
        inertia = np.asarray(self.p.inertia)
        torque = inertia*(-g.attitude_frequency**2*error-2*g.damping*g.attitude_frequency*omega) + np.cross(omega,inertia*omega)
        # NED down positive: positive down position/velocity calls for more lift.
        collective = self.p.mass*(9.81+g.height_kp*state[2]+g.height_kd*state[5])/max(R[2,2],.5)
        requested = self.inverse @ np.r_[collective,torque]
        thrust = np.clip(requested,0,self.p.max_thrust)
        return np.sqrt(thrust/self.p.max_thrust), bool(np.any(abs(requested-thrust)>1e-10))


def angles(states):
    R = np.array([rotation(q) for q in np.asarray(states)[:,6:10]])
    return np.degrees(np.column_stack((np.arctan2(R[:,2,1],R[:,2,2]),
        np.arcsin(np.clip(-R[:,2,0],-1,1)),np.arctan2(R[:,1,0],R[:,0,0]))))


def schedule(t):
    target = np.radians(10.) if 10 <= t < 14 else 0.
    torque = np.array([.12,0,0]) if 17 <= t < 17.15 else np.zeros(3)
    return target,torque


def run(controlled=True, dt=.005):
    """22 s: 10 s hover, 4 s roll command, return, then 0.15 s torque pulse."""
    if not np.isfinite(dt) or not 0 < dt <= .02:
        raise ValueError('dt must be in (0, .02]')
    quad = Quad()
    quad.state[13:] = quad.p.mass*9.81/4
    controller = Controller(quad)
    times, states, commands, targets, disturbances, saturated = [],[],[],[],[],[]
    # Insert event boundaries exactly, even when dt does not divide the schedule.
    times_grid = np.unique(np.r_[np.arange(0,22,dt),10.,14.,17.,17.15,22.])
    for i,t in enumerate(times_grid):
        target, disturbance = schedule(t)
        command, clipped = controller.command(quad.state,target) if controlled else (np.full(4,quad.hover_throttle()),False)
        times.append(t); states.append(quad.state.copy()); commands.append(command)
        targets.append(target); disturbances.append(disturbance); saturated.append(clipped)
        if i+1 < len(times_grid):
            quad.step(command,times_grid[i+1]-t,disturbance)
    return dict(time=np.array(times),state=np.array(states),throttle=np.array(commands),
                target=np.degrees(targets),disturbance=np.array(disturbances),saturated=np.array(saturated))


def settling_time(t, actual, target, start, end, tolerance=.5, dwell=.5):
    """First time within band that stays within band until phase end; >=dwell evidence."""
    mask = (t>=start)&(t<end)
    ts, errors = t[mask], abs(actual[mask]-target)
    bad = np.flatnonzero(errors>tolerance)
    index = int(bad[-1]+1) if len(bad) else 0
    if index >= len(ts) or ts[-1]-ts[index] < dwell:
        return None
    return float(ts[index]-start)


def metrics(data):
    t,states = data['time'],data['state']
    roll = angles(states)[:,0]
    command_phase = (t>=10)&(t<14)
    return_phase = (t>=14)&(t<17)
    recovery = t>=17
    return dict(hover_max_height_error_m=float(abs(states[t<=10,2]).max()),
        hover_max_angle_deg=float(abs(angles(states[t<=10])).max()),
        roll_settling_s=settling_time(t,roll,10,10,14),
        roll_overshoot_deg=float(max(0,roll[command_phase].max()-10)),
        return_settling_s=settling_time(t,roll,0,14,17),
        return_overshoot_deg=float(max(0,-roll[return_phase].min())),
        disturbance_peak_roll_deg=float(abs(roll[recovery]).max()),
        recovery_settling_s=settling_time(t,roll,0,17.15,22.00001),
        final_height_change_m=float(-states[-1,2]),max_height_excursion_m=float(abs(states[:,2]).max()),
        final_horizontal_displacement_m=float(np.linalg.norm(states[-1,:2])),
        saturated_samples=int(data['saturated'].sum()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'outputs/control')
    parser.add_argument('--no-visuals',action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    datasets = {name:run(controlled) for name,controlled in [('controlled',True),('uncontrolled',False)]}
    report = dict(parameters=asdict(Quad().p),gains=asdict(Gains()),dt_s=.005,
        limitation='Simulation only; provisional parameters, perfect state feedback, no horizontal position control or ground contact.',
        comparison='Same initial hover and torque disturbance. Uncontrolled uses fixed hover throttle and ignores angle commands.',
        settling_definition='Within +/-0.5 deg through phase end, with at least 0.5 s remaining. Recovery measured from pulse end.',
        results={name:metrics(data) for name,data in datasets.items()})
    for name,data in datasets.items():
        np.savez(args.output/f'{name}.npz',**data)
        table = np.column_stack((data['time'],data['target'],angles(data['state']),data['state'],data['throttle'],data['disturbance'],data['saturated']))
        np.savetxt(args.output/f'{name}.csv',table,delimiter=',',comments='',header='time,target_roll,roll,pitch,yaw,north,east,down,vn,ve,vd,qw,qx,qy,qz,p,q,r,T_FR,T_RR,T_RL,T_FL,u_FR,u_RR,u_RL,u_FL,tau_x,tau_y,tau_z,saturated')
    (args.output/'metrics.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report['results'],indent=2))
    if not args.no_visuals:
        from control_visuals import export
        export(datasets,args.output)


if __name__ == '__main__': main()
