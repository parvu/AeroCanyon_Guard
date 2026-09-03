"""Downsample the full wind grid into a small JSON the browser can fetch
once and render as a sparse arrow field (see web_viewer/index.html's
wind-vector-field code). The full grid (60x40x24 = 57,600 cells) is far
too many points to render individually -- this samples every 6th cell in
x, every 5th in y, at two altitude layers near typical flight altitude
(~10m, ~25m), giving a few hundred points.

Run whenever data/wind_grid.npy changes (e.g. after
`python3 -m aerocanyon.canyon_field`):
    python3 -m aerocanyon.export_wind_viz
"""
import json
import pathlib

import numpy as np

from . import canyon_geometry as cg
from .canyon_field import WindGrid

DATA_DIR = pathlib.Path(__file__).resolve().parents[1] / 'data'
OUT_PATH = (pathlib.Path(__file__).resolve().parents[3]
           / 'web_viewer' / 'wind_field_viz.json')


def export(data_dir=DATA_DIR, out_path=OUT_PATH,
          z_targets=(cg.GROUND_Z + 10.0, cg.GROUND_Z + 25.0),
          x_stride=6, y_stride=5):
    grid = WindGrid.load(str(data_dir))

    zs = sorted({int(round((z - grid.origin[2]) / grid.spacing[2]))
                for z in z_targets})
    zs = [z for z in zs if 0 <= z < grid.shape[2]]

    points = []
    for ix in range(0, grid.shape[0], x_stride):
        for iy in range(0, grid.shape[1], y_stride):
            for iz in zs:
                p_enu = grid.origin + np.array([ix, iy, iz]) * grid.spacing
                v = grid.field[ix, iy, iz]
                points.append({
                    'p': [round(float(x), 2) for x in p_enu],
                    'v': [round(float(x), 3) for x in v],
                })

    out = {'u_ref': grid.meta['u_ref'], 'points': points}
    out_path.write_text(json.dumps(out))
    return len(points)


def main():
    n = export()
    print(f'wrote {n} points to {OUT_PATH}')


if __name__ == '__main__':
    main()
