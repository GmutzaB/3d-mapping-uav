"""2D periodic incompressible Navier–Stokes propeller forcing demo."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).parent / ".mplconfig"))
import numpy as np


def propeller(throttle):
    """Empirical static thrust estimate, independent of the flow solver."""
    rho, diameter = 1.225, 0.127
    rpm = 2200 * 14.8 * 0.80 * throttle
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
    args = parser.parse_args()
    if not (0 <= args.throttle <= 1 and 0 < args.seconds <= 60
            and 24 <= args.grid <= 256 and args.grid % 2 == 0
            and np.isfinite(args.viscosity) and args.viscosity > 0):
        parser.error("Require throttle 0–1, seconds (0,60], even grid 24–256, positive finite viscosity")
    flow = Flow(args.grid, viscosity=args.viscosity)
    estimates = propeller(args.throttle)
    # Smoothed strip models a rotor cross-section. Thickness is resolved on the grid.
    thickness = max(0.02, 2*flow.dx)
    strip = np.exp(-(flow.X/0.0635)**8 - ((flow.Y-0.12)/thickness)**2)
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
    summary = dict(estimates, simulated_seconds=t, steps=steps,
                   throttle=args.throttle, grid=args.grid, effective_viscosity_m2_s=flow.nu,
                   max_divergence_per_s=float(np.max(abs(flow.divergence()))))
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), layout="constrained")
    speed = np.hypot(flow.u, flow.v)
    for ax, field, title, unit in zip(axes[:2], [speed, pressure],
            ["2D airflow speed", "Zero-mean pressure"], ["m/s", "Pa"]):
        im = ax.pcolormesh(flow.X, flow.Y, field, shading="auto", cmap="viridis")
        fig.colorbar(im, ax=ax, label=unit)
        ax.plot([-0.0635, 0.0635], [0.12, 0.12], "r-", lw=3)
        ax.set(xlabel="x [m]", ylabel="y [m]", title=title, aspect="equal")
    if speed.max() > 0:
        axes[0].streamplot(flow.x, flow.x, flow.u, flow.v, color="white", density=0.8, linewidth=0.5)
    throttles = np.linspace(0, 1, 101)
    axes[2].plot(throttles*100, [propeller(s)["total_quad_thrust_N"] for s in throttles])
    axes[2].set(xlabel="Throttle [%]", ylabel="Estimated quad thrust [N]", title="Empirical thrust estimate")
    axes[2].grid(alpha=0.3)
    fig.suptitle("3D Mapping UAV — 2D periodic flow model")
    fig.savefig(args.output / "simulation.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f"Results saved in {args.output.resolve()}")


if __name__ == "__main__":
    main()
