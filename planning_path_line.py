import numpy as np

def get_path_line(cx, cy, gx, gy):
    lx = np.arange(cx, gx, 1, dtype=float)
    ly = (cy-gy)/(cx-gx) * (lx-cx) + cy
    return lx, ly
