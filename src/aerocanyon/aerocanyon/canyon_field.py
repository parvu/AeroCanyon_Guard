"""Urban canyon wind field: offline generation, runtime lookup, turbulence.

The field is composed from published urban-canyon aerodynamics rather than
solved: a log-law vertical profile, channeling speedup through the throat
between the tower rows, and recirculation behind the tower corners. WP1 of
the funded project replaces generate() with real CFD output; the .npy plus
metadata JSON is the contract, and WindGrid does not care which produced it.

Turbulence is deliberately NOT baked into the grid -- DrydenGust adds it at
runtime. That keeps the stored field 3D instead of 4D and lets turbulence
intensity vary per trial without regenerating anything.

All vectors are Gazebo ENU, m/s.
"""
import json
import pathlib

import numpy as np

from . import canyon_geometry as cg

KARMAN = 0.41


def log_law(z, u_ref=10.0, z_ref=30.0, z0=1.0):
    """Neutral-stability logarithmic wind profile.

    z is height ABOVE GROUND (AGL), not absolute Gazebo world z -- see
    canyon_geometry.GROUND_Z. z0 = 1.0 m is the standard roughness length
    for a dense urban centre. Below z0 the profile is undefined, so it
    clamps to zero rather than returning a negative wind speed.
    """
    if z <= z0:
        return 0.0
    return u_ref * np.log(z / z0) / np.log(z_ref / z0)


def _channeling(y, z):
    """Speedup factor for flow squeezed between the two tower rows.

    z is height ABOVE GROUND (AGL), matching log_law -- see
    canyon_geometry.GROUND_Z. Peaks on the canyon axis and decays to 1.0
    (no effect) outside the throat. Only applies below the roofline;
    above it the flow is not channelled.
    """
    w = cg.CANYON_HALF_WIDTH
    if abs(y) > 3.0 * w:
        return 1.0
    roof = max(b.sz for b in cg.BUILDINGS)
    height_factor = np.clip(1.0 - z / roof, 0.0, 1.0)
    return 1.0 + 0.6 * height_factor * np.exp(-(y / w) ** 2)


def _recirculation(p):
    """Corner separation: a lateral+vertical eddy in each tower's lee.

    Modelled as a rotational contribution centred just downstream of each
    tower, decaying over one building depth. This is what makes the corners
    hazardous and is the disturbance the PINN has to learn.
    """
    v = np.zeros(3)
    for b in cg.BUILDINGS:
        centre = np.array([b.cx + b.sx * 0.75, b.cy, cg.GROUND_Z + b.sz * 0.5])
        r = p - centre
        scale = np.array([b.sx, b.sy, b.sz])
        d2 = float(np.sum((r / scale) ** 2))
        if d2 > 4.0:
            continue
        strength = 3.0 * np.exp(-d2)
        sign = np.sign(b.cy) or 1.0
        # Swirl in the plane across the canyon, plus downwash on the lee side.
        v += strength * np.array([-0.4, -sign * 0.5, -0.3])
    return v


def generate(nx=60, ny=40, nz=24):
    """Build the 3D wind grid over the canyon and its surroundings.

    Returns (field, meta) where field has shape (nx, ny, nz, 3).
    """
    xs_lo, xs_hi = cg.CANYON_ENTRY[0] - 20.0, cg.CANYON_EXIT[0] + 20.0
    ys_lo, ys_hi = -150.0, 150.0
    # Absolute Gazebo z, spanning canyon_geometry.GROUND_Z (ground) to
    # GROUND_Z + 100 (well above the tallest building) -- log_law/
    # _channeling below convert back to AGL height internally.
    zs_lo, zs_hi = cg.GROUND_Z, cg.GROUND_Z + 100.0

    xs = np.linspace(xs_lo, xs_hi, nx)
    ys = np.linspace(ys_lo, ys_hi, ny)
    zs = np.linspace(zs_lo, zs_hi, nz)

    field = np.zeros((nx, ny, nz, 3))
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                p = np.array([x, y, z])
                d, _ = cg.distance_and_normal(p)
                if d <= 0.0:
                    continue  # inside a building: no flow
                agl = z - cg.GROUND_Z
                base = log_law(agl) * _channeling(y, agl)
                # Prevailing wind along +x, down the canyon axis.
                v = np.array([base, 0.0, 0.0]) + _recirculation(p)
                # Blend to zero at the walls (no-slip).
                v *= np.clip(d / 3.0, 0.0, 1.0)
                field[i, j, k] = v

    meta = {
        'origin': [xs_lo, ys_lo, zs_lo],
        'spacing': [(xs_hi - xs_lo) / (nx - 1),
                    (ys_hi - ys_lo) / (ny - 1),
                    (zs_hi - zs_lo) / (nz - 1)],
        'shape': [nx, ny, nz],
        'u_ref': 10.0,
        'z0': 1.0,
    }
    return field, meta


class WindGrid:
    """Trilinear lookup into a generated or CFD-produced wind grid."""

    def __init__(self, field, meta):
        self.field = np.asarray(field, dtype=float)
        self.origin = np.array(meta['origin'], dtype=float)
        self.spacing = np.array(meta['spacing'], dtype=float)
        self.shape = np.array(meta['shape'], dtype=int)
        self.meta = meta

    @classmethod
    def load(cls, data_dir):
        d = pathlib.Path(data_dir)
        field = np.load(d / 'wind_grid.npy')
        meta = json.loads((d / 'wind_grid.json').read_text())
        return cls(field, meta)

    def save(self, data_dir):
        d = pathlib.Path(data_dir)
        d.mkdir(parents=True, exist_ok=True)
        np.save(d / 'wind_grid.npy', self.field)
        (d / 'wind_grid.json').write_text(json.dumps(self.meta, indent=2))

    def at(self, p_enu):
        """Wind velocity at an ENU point. Clamps outside the grid."""
        f = (np.asarray(p_enu, dtype=float) - self.origin) / self.spacing
        f = np.clip(f, 0.0, self.shape - 1.0)
        i0 = np.floor(f).astype(int)
        i1 = np.minimum(i0 + 1, self.shape - 1)
        t = f - i0

        out = np.zeros(3)
        for bx in (0, 1):
            for by in (0, 1):
                for bz in (0, 1):
                    w = ((t[0] if bx else 1 - t[0])
                         * (t[1] if by else 1 - t[1])
                         * (t[2] if bz else 1 - t[2]))
                    if w == 0.0:
                        continue
                    idx = (i1[0] if bx else i0[0],
                           i1[1] if by else i0[1],
                           i1[2] if bz else i0[2])
                    out += w * self.field[idx]
        return out


class DrydenGust:
    """Dryden turbulence as a first-order Markov process per axis.

    The full Dryden spectrum needs a second-order filter on the lateral
    axes; a first-order approximation matches the dominant time constant,
    which is what governs how the fractional-memory term behaves. That is
    the property under test here.

    ponytail: first-order Dryden approximation. Upgrade to the full
    second-order lateral filter only if spectral fidelity is ever claimed
    in a paper.
    """

    # length_scale sets the correlation time via tau = length_scale / airspeed.
    # It was 200 m, which at this vehicle's ~8 m/s cruise gives tau ~= 25 s --
    # so the "gust" was really a slowly wandering steady bias, not turbulence.
    # Measured consequences: raising sigma barely moved the unsteady content
    # (detrended gust std went 0.15 -> 0.22 m/s for a 2.7x sigma increase)
    # because the energy all sat below PX4's position-integrator bandwidth,
    # and a large slow bias acted as a random head/tailwind that stopped the
    # vehicle completing the transit at all (130 m in 216 s, against 300 m in
    # 99 s before). 25 m gives tau ~= 3 s at cruise: genuinely unsteady, which
    # is both the regime a feedforward estimate can uniquely help with and the
    # one the fractional-memory term exists to capture.
    def __init__(self, dt, sigma=2.5, length_scale=25.0, seed=0):
        self.dt = float(dt)
        self.sigma = float(sigma)
        self.length_scale = float(length_scale)
        self.rng = np.random.default_rng(seed)
        self.state = np.zeros(3)

    def step(self, airspeed):
        """Advance one timestep. Faster flight decorrelates gusts sooner."""
        v = max(float(airspeed), 1.0)
        tau = self.length_scale / v
        a = np.exp(-self.dt / tau)
        noise = self.rng.normal(0.0, 1.0, 3)
        self.state = a * self.state + self.sigma * np.sqrt(1.0 - a * a) * noise
        return self.state.copy()


if __name__ == '__main__':
    field, meta = generate()
    out = pathlib.Path(__file__).resolve().parents[1] / 'data'
    WindGrid(field, meta).save(out)
    speeds = np.linalg.norm(field, axis=-1)
    print(f'wrote {out}/wind_grid.npy shape={field.shape} '
          f'max={speeds.max():.2f} m/s mean={speeds.mean():.2f} m/s')
