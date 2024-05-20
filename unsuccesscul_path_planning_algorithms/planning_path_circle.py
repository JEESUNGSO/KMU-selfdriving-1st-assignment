import numpy as np

def get_turning_circle(c_pos, cdirection, turning_radius, density):
    cdir_vertical = cdirection - np.pi / 2
    # 오른쪽 회전
    turning_center_R = c_pos + np.array([np.cos(cdir_vertical), np.sin(cdir_vertical)]) * turning_radius
    turning_points_R = np.array([turning_center_R + np.array([np.cos(theta), np.sin(theta)]) \
                                 * turning_radius for theta in np.linspace(0, np.pi * 2, density)])
    # 왼쪽 회전
    turning_center_L = c_pos - np.array([np.cos(cdir_vertical), np.sin(cdir_vertical)]) * turning_radius
    turning_points_L = np.array([turning_center_L + np.array([np.cos(theta), np.sin(theta)]) \
                                 * turning_radius for theta in np.linspace(0, np.pi * 2, density)])

    return turning_points_R, turning_points_L, turning_center_R, turning_center_L


def get_tangent_lines(c1, r1, c2, r2):
    def compute_tangents(x1, y1, r1, x2, y2, r2, outer=True):
        d = np.hypot(x2 - x1, y2 - y1)
        if d < abs(r1 - r2):
            return []

        tangents = []
        if outer:
            r_diff = r1 - r2
        else:
            r_diff = r1 + r2

        angle_between_centers = np.arctan2(y2 - y1, x2 - x1)
        angle_offset = np.arccos(r_diff / d)

        for sign in [-1, 1]:
            theta = angle_between_centers + sign * angle_offset
            if outer:
                t1 = (x1 + r1 * np.cos(theta), y1 + r1 * np.sin(theta))
                t2 = (x2 + r2 * np.cos(theta), y2 + r2 * np.sin(theta))
            else:
                t1 = (x1 + r1 * np.cos(theta), y1 + r1 * np.sin(theta))
                t2 = (x2 - r2 * np.cos(theta), y2 - r2 * np.sin(theta))

            tangents.append((t1, t2))
        return tangents

    x1, y1 = c1
    x2, y2 = c2

    outer_tangents = compute_tangents(x1, y1, r1, x2, y2, r2, outer=True)
    return outer_tangents


def get_path_circle(cx, cy, cdirection, gx, gy, gdirection, turning_radius):
    density = 100

    start_pos = np.array([cx, cy])
    end_pos = np.array([gx, gy])

    start_turning_R, start_turning_L, start_center_R, start_center_L = get_turning_circle(start_pos, cdirection,
                                                                                          turning_radius, density)
    end_turning_R, end_turning_L, end_center_R, end_center_L = get_turning_circle(end_pos, gdirection, turning_radius,
                                                                                  density)

    start_turning_centers = [start_center_R, start_center_L]
    end_turning_centers = [end_center_R, end_center_L]

    tangents = []
    for s_center in start_turning_centers:
        for e_center in end_turning_centers:
            tangents.extend(get_tangent_lines(s_center, turning_radius, e_center, turning_radius))

    path_x, path_y = [], []
    if tangents:
        for (sx, sy), (ex, ey) in tangents:
            path_x.extend([sx, ex])
            path_y.extend([sy, ey])

    return path_x, path_y
