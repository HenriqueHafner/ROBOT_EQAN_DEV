"""
Created on 2025
@author: Henrique Hafner Ferreira @company: OuroNova Brazil
Trajectory generation for jerk-controlled motion using symmetric 7-phase approximation.
See also: https://en.wikipedia.org/wiki/Jerk_(physics)
This module is part of the Tankbotics motion control library.
"""

def compute_jerk_for_unit_trajectory(maneuver_time):
    if maneuver_time <= 0:
        return 0.0
    t1 = maneuver_time / 5
    t2 = maneuver_time - 2 * t1
    if t2 < 0:
        t2 = 0  # Avoid negative for short times
    numerator = 1.0
    denominator = 2 * (t1**3 + t1**2 * t2 + 0.5 * t1 * t2**2 + (1/6) * t2**3)
    jerk = numerator / denominator
    return jerk

def compute_template_positions(maneuver_time, iteration_period, jerk):
    if maneuver_time <= 0 or iteration_period <= 0:
        return []
    interval = maneuver_time / 2
    t1 = interval / 5
    t2 = interval - 2 * t1
    if t2 < 0:
        t2 = 0

    positions_unity_displacement = []
    time = 0.0
    position = 0.0
    velocity = 0.0
    acceleration = 0.0

    while time < interval:
        if time < t1:  # phase i11
            j = jerk
        elif time < t1 + t2:  # phase i12
            j = 0.0
        else:
            j = -jerk  # phase i13
        acceleration += j * iteration_period
        velocity += acceleration * iteration_period
        position += velocity * iteration_period
        positions_unity_displacement.append(position)
        time += iteration_period

    # Add exact endpoint
    positions_unity_displacement.append(position)

    final_template_position = positions_unity_displacement[-1]
    if final_template_position <= 0:
        return []

    positions_half = []
    for p in positions_unity_displacement:
        normalized_position = p / (2 * final_template_position)
        positions_half.append(normalized_position)

    mirrored_positions = []
    for p in reversed(positions_half):
        mirrored_position = 1.0 - p
        mirrored_positions.append(mirrored_position)

    positions_full = positions_half + mirrored_positions[1:]  # Avoid duplicate at 0.5

    if any(p < 0.0 or p > 1.0 for p in positions_full):
        return []

    return positions_full

def scale_and_offset_trajectory(positions_template, start_position, end_position):
    scale_factor = end_position - start_position
    positions = [
        start_position + p * scale_factor
        for p in positions_template
    ]
    return positions

def get_manuever_positions(start_position, end_position, maneuver_time, iteration_period):
    jerk_template = compute_jerk_for_unit_trajectory(maneuver_time)
    positions_template = compute_template_positions(maneuver_time, iteration_period, jerk_template)
    positions = scale_and_offset_trajectory(positions_template, start_position, end_position)
    return positions