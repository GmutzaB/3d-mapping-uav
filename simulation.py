"""2D periodic incompressible Navier–Stokes propeller forcing demo."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).parent / ".mplconfig"))
import numpy as np


def propeller(throttle, motor_kv=1500, battery_cells=6, prop_diameter_in=7):
    """Empirical static thrust estimate, independent of the flow solver."""
    rho, diameter = 1.225, prop_diameter_in * 0.0254
    rpm = motor_kv * (3.7 * battery_cells) * 0.80 * throttle
    thrust = 0.11 * rho * (rpm / 60)**2 * diameter**4
    area = np.pi * diameter**2 / 4
    return dict(rpm=rpm, thrust_per_motor_N=thrust,
                total_quad_thrust_N=4 * thrust,
                equivalent_supported_mass_kg=4 * thrust / 9.81,
                ideal_disk_pressure_jump_Pa=thrust / area,
                ideal_induced_velocity_m_s=float(np.sqrt(thrust / (2 * rho * area))))


class Flow:
    """Fourier derivatives, 2/3 filtering, pressure projection, explicit RK2."""
    def __init__(self, n=64, length=0.8, viscosity=0.001):
        self.n, self.length, self.nu = n, length, viscosity
        self.dx = length / n
        self.x = np.arange(n) * self.dx - length / 2
        self.X, self.Y = np.meshgrid(self.x, self.x)
        k = 2 * np.pi * np.fft.fftfreq(n, d=self.dx)
        self.kx, self.ky = np.meshgrid(k, k)
        self.k2 = self.kx**2 + self.ky**2
        self.safe_k2 = np.where(self.k2 == 0, 1, self.k2)
        modes = np.fft.fftfreq(n) * n
        mx, my = np.meshgrid(modes, modes)
        self.mask = (abs(mx) < n / 3) & (abs(my) < n / 3)
        self.u = np.zeros((n, n))
        self.v = self.u.copy()
        self.fx = self.u.copy()
        self.fy = self.u.copy()

    def derivative(self, a, k):
        return np.fft.ifft2(1j * k * np.fft.fft2(a)).real

    def rhs(self, u, v):
        uh, vh = np.fft.fft2(u), np.fft.fft2(v)
        ax = np.fft.fft2(-u*self.derivative(u, self.kx)
                          -v*self.derivative(u, self.ky) + self.fx) * self.mask
        ay = np.fft.fft2(-u*self.derivative(v, self.kx)
                          -v*self.derivative(v, self.ky) + self.fy) * self.mask
        # Laplacian(p) = rho * divergence(non-pressure acceleration).
        ph = -1.225 * 1j * (self.kx*ax + self.ky*ay) / self.safe_k2
        ph[0, 0] = 0
        ax = ax - 1j*self.kx*ph/1.225 - self.nu*self.k2*uh
        ay = ay - 1j*self.ky*ph/1.225 - self.nu*self.k2*vh
        return np.fft.ifft2(ax).real, np.fft.ifft2(ay).real, np.fft.ifft2(ph).real

    def step(self, dt):
        a, b, _ = self.rhs(self.u, self.v)
        c, d, _ = self.rhs(self.u + dt*a, self.v + dt*b)
        self.u += dt*(a+c)/2
        self.v += dt*(b+d)/2

    def divergence(self):
        return self.derivative(self.u, self.kx) + self.derivative(self.v, self.ky)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--throttle", type=float, default=0.5)
    parser.add_argument("--seconds", type=float, default=0.15)
    parser.add_argument("--grid", type=int, default=64)
    parser.add_argument("--viscosity", type=float, default=0.001,
                        help="Effective kinematic viscosity [m²/s], deliberately above air viscosity")
    parser.add_argument("--output", type=Path, default=Path("outputs"))
    parser.add_argument("--prop-diameter-in", type=float, default=7)
    parser.add_argument("--prop-pitch-in", type=float, default=4.5, help="Recorded only; pitch effects are not modeled")
    parser.add_argument("--motor-kv", type=float, default=1500)
    parser.add_argument("--battery-cells", type=int, default=6)
    parser.add_argument("--battery-mah", type=float, default=4500)
    parser.add_argument("--battery-c", type=float, default=50)
    parser.add_argument("--force-thickness", type=float, default=0.05,
                        help="Fixed forcing thickness [m], shared across grid comparisons")
    args = parser.parse_args()
    if not (all(np.isfinite(v) and v > 0 for v in
                (args.prop_diameter_in, args.prop_pitch_in, args.motor_kv, args.battery_mah, args.battery_c, args.force_thickness))
            and args.battery_cells > 0):
        parser.error("Hardware values and force thickness must be positive and finite")
    if not (0 <= args.throttle <= 1 and 0 < args.seconds <= 60
            and 24 <= args.grid <= 256 and args.grid % 2 == 0
            and np.isfinite(args.viscosity) and args.viscosity > 0):
        parser.error("Require throttle 0–1, seconds (0,60], even grid 24–256, positive finite viscosity")
    flow = Flow(args.grid, viscosity=args.viscosity)
    estimates = propeller(args.throttle, args.motor_kv, args.battery_cells, args.prop_diameter_in)
    # Hold the physical forcing width fixed when changing grid resolution.
    thickness = args.force_thickness
    radius = args.prop_diameter_in * 0.0254 / 2
    strip = np.exp(-(flow.X/radius)**8 - ((flow.Y-0.12)/thickness)**2)
    flow.fy = -estimates["ideal_disk_pressure_jump_Pa"] / (1.225*thickness*np.sqrt(np.pi)) * strip
    # A periodic domain requires a balancing background to avoid bulk acceleration.
    flow.fy -= flow.fy.mean()
    flow.fy = np.fft.ifft2(np.fft.fft2(flow.fy)*flow.mask).real
    t, steps = 0.0, 0
    while t < args.seconds:
        rate = np.max(abs(flow.u) + abs(flow.v))
        dt = min(0.001, 0.2*flow.dx/max(rate, 1e-8),
                 0.2*flow.dx**2/flow.nu, args.seconds-t)
        flow.step(dt)
        if not np.isfinite(flow.u).all() or not np.isfinite(flow.v).all():
            raise RuntimeError("Flow became unstable; reduce duration or increase viscosity")
        t += dt
        steps += 1
    _, _, pressure = flow.rhs(flow.u, flow.v)
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output / "flow.npz", x=flow.x, y=flow.x,
                        u=flow.u, v=flow.v, pressure_Pa=pressure)
    speed = np.hypot(flow.u, flow.v)
    # One row per grid point for Numbers, Excel, and other spreadsheet apps.
    np.savetxt(args.output / "flow.csv",
               np.column_stack([field.ravel() for field in
                                (flow.X, flow.Y, flow.u, flow.v, speed, pressure)]),
               delimiter=",", comments="",
               header="x_m,y_m,u_m_s,v_m_s,speed_m_s,pressure_Pa")
    summary = dict(estimates, simulated_seconds=t, steps=steps,
                   motor_kv=args.motor_kv, battery_cells=args.battery_cells,
                   battery_nominal_voltage_V=3.7*args.battery_cells,
                   battery_capacity_mAh=args.battery_mah, battery_c_rating=args.battery_c,
                   battery_nominal_energy_Wh=3.7*args.battery_cells*args.battery_mah/1000,
                   propeller_diameter_m=2*radius, propeller_diameter_in=args.prop_diameter_in,
                   propeller_pitch_in=args.prop_pitch_in, pitch_effects_modeled=False, thrust_coefficient=0.11,
                   loaded_rpm_factor=0.8, force_thickness_m=thickness,
                   peak_speed_m_s=float(speed.max()),
                   rms_speed_m_s=float(np.sqrt(np.mean(speed**2))),
                   pressure_min_Pa=float(pressure.min()), pressure_max_Pa=float(pressure.max()),
                   throttle=args.throttle, grid=args.grid, effective_viscosity_m2_s=flow.nu,
                   max_divergence_per_s=float(np.max(abs(flow.divergence()))))
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), layout="constrained")
    for ax, field, title, unit in zip(axes[:2], [speed, pressure],
            ["2D airflow speed", "Zero-mean pressure"], ["m/s", "Pa"]):
        im = ax.pcolormesh(flow.X, flow.Y, field, shading="auto", cmap="viridis")
        fig.colorbar(im, ax=ax, label=unit)
        ax.plot([-radius, radius], [0.12, 0.12], "r-", lw=3)
        ax.set(xlabel="x [m]", ylabel="y [m]", title=title, aspect="equal")
    if speed.max() > 0:
        axes[0].streamplot(flow.x, flow.x, flow.u, flow.v, color="white", density=0.8, linewidth=0.5)
    throttles = np.linspace(0, 1, 101)
    axes[2].plot(throttles*100, [propeller(s, args.motor_kv, args.battery_cells, args.prop_diameter_in)["total_quad_thrust_N"] for s in throttles])
    axes[2].set(xlabel="Throttle [%]", ylabel="Estimated quad thrust [N]", title="Empirical thrust estimate")
    axes[2].grid(alpha=0.3)
    fig.suptitle("3D Mapping UAV — 2D periodic flow model\n"
                 f"Grid {args.grid}×{args.grid} | t={t:g} s | throttle={args.throttle:.0%} | "
                 f"viscosity={flow.nu:g} m²/s | forcing width={thickness:g} m\n"
                 f"Provisional: {args.motor_kv:g} KV, {args.battery_cells}S "
                 f"{args.battery_mah:g} mAh {args.battery_c:g}C | {args.prop_diameter_in:g}×{args.prop_pitch_in:g} in prop (pitch not modeled), Ct=0.11")
    fig.savefig(args.output / "simulation.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f"Results saved in {args.output.resolve()}")
    print("Open simulation.png in Preview or flow.csv in Numbers / Excel.")


if __name__ == "__main__":
    main()
