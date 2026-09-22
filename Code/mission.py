import lidar
import matplotlib.pyplot as plt
import math
import json
import time
import numpy as np

CELL_SIZE = 0.05


def scan_convert(data):
    points = []
    for (quality, angle, distance) in data:
        if distance > 0 and quality >= 10:
            distance_m = distance / 1000.0
            x = distance_m * math.cos(math.radians(angle))
            y = distance_m * math.sin(math.radians(angle))
            points.append((x, y))
    return points


def detect_nearby_objects(data):
    objectdet = {
        "front": False,
        "back": False,
        "left": False,
        "right": False
    }

    for (quality, angle, distance) in data:
        if distance < 1000:
            if angle > 340 or angle < 20:
                objectdet["front"] = True
            elif 80 < angle < 100:
                objectdet["left"] = True
            elif 260 < angle < 280:
                objectdet["right"] = True

        if distance < 500:
            if 160 < angle < 200:
                objectdet["back"] = True

    return objectdet


def read_latest_pose():
    with open("/tmp/pixhawk_pose.json", "r") as f:
        latest_pose = json.load(f)
    return latest_pose["yaw"], latest_pose["x"], latest_pose["y"]


def world_to_cell(x, y, cell_size=CELL_SIZE):
    return (int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))


def transform_points(local_points, theta, x, y):
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (R @ local_points.T).T + np.array([x, y])


def score_pose(local_points, theta, x, y, occupied_cells):
    world_points = transform_points(local_points, theta, x, y)
    score = 0
    for px, py in world_points:
        if world_to_cell(px, py) in occupied_cells:
            score += 1
    return score


def search_poses(local_points, center_theta, center_x, center_y, occupied_cells,
                  xy_range, xy_step, theta_range, theta_step):
    best_score = -1
    best_pose = (center_theta, center_x, center_y)

    for dtheta in np.arange(-theta_range, theta_range + theta_step, theta_step):
        for dx in np.arange(-xy_range, xy_range + xy_step, xy_step):
            for dy in np.arange(-xy_range, xy_range + xy_step, xy_step):
                theta = center_theta + dtheta
                x = center_x + dx
                y = center_y + dy
                score = score_pose(local_points, theta, x, y, occupied_cells)
                if score > best_score:
                    best_score = score
                    best_pose = (theta, x, y)

    return best_pose, best_score


def search_best_pose_coarse_to_fine(local_points, guess_theta, guess_x, guess_y, occupied_cells):
    coarse_pose, coarse_score = search_poses(
        local_points, guess_theta, guess_x, guess_y, occupied_cells,
        xy_range=0.3, xy_step=0.10,
        theta_range=0.15, theta_step=0.05
    )

    fine_pose, fine_score = search_poses(
        local_points, coarse_pose[0], coarse_pose[1], coarse_pose[2], occupied_cells,
        xy_range=0.10, xy_step=0.02,
        theta_range=0.05, theta_step=0.01
    )

    return fine_pose, fine_score


def get_map_bounds(occupied_cells, cell_size=CELL_SIZE):
    if not occupied_cells:
        return None
    xs = [c[0] for c in occupied_cells]
    ys = [c[1] for c in occupied_cells]
    min_x = min(xs) * cell_size
    max_x = (max(xs) + 1) * cell_size
    min_y = min(ys) * cell_size
    max_y = (max(ys) + 1) * cell_size
    return min_x, max_x, min_y, max_y


def build_dense_grid(occupied_cells, bounds, cell_size=CELL_SIZE):
    min_x, max_x, min_y, max_y = bounds
    min_cx = int(math.floor(min_x / cell_size))
    max_cx = int(math.floor(max_x / cell_size))
    min_cy = int(math.floor(min_y / cell_size))
    max_cy = int(math.floor(max_y / cell_size))
    width = max_cx - min_cx + 1
    height = max_cy - min_cy + 1

    grid = np.zeros((width, height), dtype=bool)
    for (cx, cy) in occupied_cells:
        grid[cx - min_cx, cy - min_cy] = True

    return grid, min_cx, min_cy, width, height


def score_pose_against_grid(rotated_points, x, y, grid, min_cx, min_cy, width, height, cell_size=CELL_SIZE):
    world_x = rotated_points[:, 0] + x
    world_y = rotated_points[:, 1] + y
    cx = np.floor(world_x / cell_size).astype(int) - min_cx
    cy = np.floor(world_y / cell_size).astype(int) - min_cy
    valid = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
    if not valid.any():
        return 0
    return int(grid[cx[valid], cy[valid]].sum())


def global_search_pose(local_points, occupied_cells, guess_theta, xy_step=0.5,
                        theta_search_range=math.radians(30), theta_step=math.radians(10),
                        max_points=60, cell_size=CELL_SIZE):
    bounds = get_map_bounds(occupied_cells, cell_size)
    if bounds is None:
        return None, -1, None, -1

    grid, min_cx, min_cy, width, height = build_dense_grid(occupied_cells, bounds, cell_size)

    if len(local_points) > max_points:
        idx = np.linspace(0, len(local_points) - 1, max_points).astype(int)
        points = local_points[idx]
    else:
        points = local_points

    min_x, max_x, min_y, max_y = bounds
    best_score = -1
    second_best_score = -1
    best_pose = None
    second_best_pose = None

    theta_min = guess_theta - theta_search_range
    theta_max = guess_theta + theta_search_range

    for theta in np.arange(theta_min, theta_max + theta_step, theta_step):
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]])
        rotated = (R @ points.T).T

        for x in np.arange(min_x, max_x, xy_step):
            for y in np.arange(min_y, max_y, xy_step):
                score = score_pose_against_grid(rotated, x, y, grid, min_cx, min_cy, width, height, cell_size)
                if score > best_score:
                    second_best_score = best_score
                    second_best_pose = best_pose
                    best_score = score
                    best_pose = (theta, x, y)
                elif score > second_best_score:
                    second_best_score = score
                    second_best_pose = (theta, x, y)

    return best_pose, best_score, second_best_pose, second_best_score


def mark_ray_free(free_cells, occupied_cells, x0, y0, x1, y1, cell_size=CELL_SIZE):
    dist = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(dist / (cell_size / 2)))

    for i in range(steps):
        t = i / steps
        px = x0 + t * (x1 - x0)
        py = y0 + t * (y1 - y0)
        cell = world_to_cell(px, py, cell_size)
        if cell not in occupied_cells:
            free_cells.add(cell)


def update_map(occupied_cells, free_cells, local_points, theta, x, y):
    world_points = transform_points(local_points, theta, x, y)
    for px, py in world_points:
        mark_ray_free(free_cells, occupied_cells, x, y, px, py)
        occupied_cells.add(world_to_cell(px, py))


def update_coverage(visited_cells, x, y):
    visited_cells.add(world_to_cell(x, y))


def choose_next_target(x, y, visited_cells, occupied_cells, free_cells, search_radius=2.0, cell_size=CELL_SIZE):
    current_cell = world_to_cell(x, y, cell_size)
    cell_radius = int(math.ceil(search_radius / cell_size))

    best_target = None
    best_dist = None

    for dcx in range(-cell_radius, cell_radius + 1):
        for dcy in range(-cell_radius, cell_radius + 1):
            candidate_cell = (current_cell[0] + dcx, current_cell[1] + dcy)

            if candidate_cell in visited_cells or candidate_cell in occupied_cells:
                continue
            if candidate_cell not in free_cells:
                continue

            cx = (candidate_cell[0] + 0.5) * cell_size
            cy = (candidate_cell[1] + 0.5) * cell_size
            dist = math.hypot(cx - x, cy - y)

            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_target = (cx, cy)

    return best_target


MATCH_QUALITY_THRESHOLD = 0.3
CONFIDENCE_MARGIN = 0.15

occupied_cells = set()
free_cells = set()
visited_cells = set()

theta_offset = 0.0
x_offset = 0.0
y_offset = 0.0

for data in lidar.lidar.iter_scans():
    current_scan = np.array(scan_convert(data))
    if len(current_scan) == 0:
        continue

    objectdet = detect_nearby_objects(data)

    guess_theta, guess_x, guess_y = read_latest_pose()

    corrected_guess_theta = guess_theta + theta_offset
    corrected_guess_x = guess_x + x_offset
    corrected_guess_y = guess_y + y_offset

    trust_this_result = False

    if len(occupied_cells) == 0:
        best_theta, best_x, best_y = corrected_guess_theta, corrected_guess_x, corrected_guess_y
        trust_this_result = True
    else:
        (best_theta, best_x, best_y), best_score = search_best_pose_coarse_to_fine(
            current_scan, corrected_guess_theta, corrected_guess_x, corrected_guess_y, occupied_cells
        )

        match_ratio = best_score / len(current_scan)
        if match_ratio >= MATCH_QUALITY_THRESHOLD:
            trust_this_result = True
        else:
            coarse_best_pose, coarse_best_score, coarse_second_pose, coarse_second_score = global_search_pose(
                current_scan, occupied_cells, corrected_guess_theta
            )

            if coarse_best_pose is not None:
                refined_pose, refined_score = search_poses(
                    current_scan, coarse_best_pose[0], coarse_best_pose[1], coarse_best_pose[2], occupied_cells,
                    xy_range=0.1, xy_step=0.02, theta_range=0.05, theta_step=0.01
                )

                if coarse_second_pose is not None:
                    _, refined_second_score = search_poses(
                        current_scan, coarse_second_pose[0], coarse_second_pose[1], coarse_second_pose[2],
                        occupied_cells, xy_range=0.1, xy_step=0.02, theta_range=0.05, theta_step=0.01
                    )
                else:
                    refined_second_score = 0

                global_ratio = refined_score / len(current_scan)
                margin = (refined_score - refined_second_score) / max(refined_score, 1)

                if global_ratio > match_ratio and margin >= CONFIDENCE_MARGIN:
                    best_theta, best_x, best_y = refined_pose
                    trust_this_result = True
                    print(f"  [loop closure] weak local match ({match_ratio:.2f}) — "
                          f"corrected via global search ({global_ratio:.2f}, margin {margin:.2f})")
                else:
                    print(f"  [loop closure] weak local match ({match_ratio:.2f}) — "
                          f"global search too ambiguous to trust (margin {margin:.2f}), keeping local result")

    if trust_this_result:
        theta_offset = ((best_theta - guess_theta) + math.pi) % (2 * math.pi) - math.pi
        x_offset = best_x - guess_x
        y_offset = best_y - guess_y

    update_map(occupied_cells, free_cells, current_scan, best_theta, best_x, best_y)
    update_coverage(visited_cells, best_x, best_y)

    next_target = choose_next_target(best_x, best_y, visited_cells, occupied_cells, free_cells)

    mission_state = {
        "timestamp": time.time(),
        "pose": {"x": best_x, "y": best_y, "theta": best_theta},
        "next_target": {"x": next_target[0], "y": next_target[1]} if next_target is not None else None,
        "nearby": objectdet,
        "coverage": {
            "obstacle_cells": len(occupied_cells),
            "free_cells": len(free_cells),
            "visited_cells": len(visited_cells)
        }
    }
    with open("/tmp/mission_state.json", "w") as f:
        json.dump(mission_state, f)

    print(f"pose: x={best_x:.2f}m y={best_y:.2f}m theta={math.degrees(best_theta):.1f}deg  "
          f"obstacle cells: {len(occupied_cells)}  free cells: {len(free_cells)}  "
          f"visited cells: {len(visited_cells)}  next target: {next_target}  "
          f"nearby: {objectdet}")