import numpy as np

def get_path_line(cx, cy, gx, gy):
    theta = np.arctan2(gy-cy, gx-cx)
    l = np.arange(0, np.linalg.norm(np.array([gx-cx, gy-cy])), 0.5, dtype=float)
    lx = l * np.cos(theta) + cx
    ly = l * np.sin(theta) + cy
    return lx, ly
