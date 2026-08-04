#!/usr/bin/env python3
"""Generate a Gazebo Classic world containing walls and random cylinders.

Example:
  ./generate_cylinder_world.py --size-x 40 --size-y 30 \\
      --cylinder-density 0.08 --height-min 1.0 --height-max 4.0 \\
      --seed 42 --output random_40x30.world

``--cylinder-density`` is measured in cylinders per square metre.  The
generator uses rejection sampling, so cylinders never overlap each other or
the enclosing walls.  No ceiling is generated.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path
from typing import List, Tuple
from xml.sax.saxutils import escape, quoteattr


def positive_float(value: str) -> float:
    result = float(value)
    if result <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return result


def nonnegative_float(value: str) -> float:
    result = float(value)
    if result < 0.0:
        raise argparse.ArgumentTypeError("must not be negative")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a bounded Gazebo Classic world with random cylindrical obstacles.")
    parser.add_argument("--output", type=Path, default=None,
                        help="output .world path (default: cylinders_<sizeX>_<sizeY>.world)")
    parser.add_argument("--world-name", default="random_cylinder_world",
                        help="SDF world name (default: %(default)s)")
    parser.add_argument("--size-x", type=positive_float, default=20.0,
                        help="inner map width along x in metres (default: %(default)s)")
    parser.add_argument("--size-y", type=positive_float, default=20.0,
                        help="inner map width along y in metres (default: %(default)s)")
    parser.add_argument("--cylinder-density", type=nonnegative_float, default=0.08,
                        help="cylinders per square metre (default: %(default)s)")
    parser.add_argument("--cylinder-radius", type=positive_float, default=0.30,
                        help="cylinder radius in metres (default: %(default)s)")
    parser.add_argument("--height-min", type=positive_float, default=2.5,
                        help="minimum cylinder height in metres (default: %(default)s)")
    parser.add_argument("--height-max", type=positive_float, default=6.0,
                        help="maximum cylinder height in metres (default: %(default)s)")
    parser.add_argument("--wall-height", type=positive_float, default=5.0,
                        help="wall height in metres (default: %(default)s)")
    parser.add_argument("--wall-thickness", type=positive_float, default=0.15,
                        help="wall thickness in metres (default: %(default)s)")
    parser.add_argument("--wall-clearance", type=nonnegative_float, default=0.20,
                        help="extra cylinder-to-wall clearance in metres (default: %(default)s)")
    parser.add_argument("--obstacle-clearance", type=nonnegative_float, default=0.20,
                        help="extra cylinder-to-cylinder clearance in metres (default: %(default)s)")
    parser.add_argument("--keep-clear-radius", type=nonnegative_float, default=1.5,
                        help="clear radius around the origin for vehicle spawn (default: %(default)s)")
    parser.add_argument("--seed", type=int, default=None,
                        help="random seed; set it to reproduce a map")
    parser.add_argument("--max-placement-attempts", type=int, default=5000,
                        help="attempts per cylinder before reporting an over-dense map")
    args = parser.parse_args()
    if args.height_min > args.height_max:
        parser.error("--height-min must not exceed --height-max")
    if args.max_placement_attempts <= 0:
        parser.error("--max-placement-attempts must be greater than zero")
    if args.output is None:
        args.output = Path(f"cylinders_{args.size_x:g}_{args.size_y:g}.world")
    return args


def place_cylinders(args: argparse.Namespace, rng: random.Random) -> List[Tuple[float, float, float]]:
    count = round(args.size_x * args.size_y * args.cylinder_density)
    edge_margin = args.cylinder_radius + args.wall_clearance
    min_x, max_x = -args.size_x / 2.0 + edge_margin, args.size_x / 2.0 - edge_margin
    min_y, max_y = -args.size_y / 2.0 + edge_margin, args.size_y / 2.0 - edge_margin
    if min_x > max_x or min_y > max_y:
        raise ValueError("map is too small for the cylinder radius and wall clearance")

    required_distance = 2.0 * args.cylinder_radius + args.obstacle_clearance
    required_distance_sq = required_distance * required_distance
    cylinders: List[Tuple[float, float, float]] = []
    for index in range(count):
        for _ in range(args.max_placement_attempts):
            x = rng.uniform(min_x, max_x)
            y = rng.uniform(min_y, max_y)
            if math.hypot(x, y) < args.keep_clear_radius + args.cylinder_radius:
                continue
            if any((x - old_x) ** 2 + (y - old_y) ** 2 < required_distance_sq
                   for old_x, old_y, _ in cylinders):
                continue
            cylinders.append((x, y, rng.uniform(args.height_min, args.height_max)))
            break
        else:
            raise ValueError(
                f"could only place {len(cylinders)} of {count} cylinders; reduce "
                "--cylinder-density/radius/clearance or enlarge the map")
    return cylinders


def box_model(name: str, x: float, y: float, sx: float, sy: float, height: float) -> str:
    return f'''    <model name='{escape(name)}'>
      <static>1</static>
      <pose>{x:.6f} {y:.6f} {height / 2.0:.6f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>{sx:.6f} {sy:.6f} {height:.6f}</size></box></geometry></collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.6f} {sy:.6f} {height:.6f}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Bricks</name></script></material>
        </visual>
      </link>
    </model>'''


def cylinder_model(index: int, x: float, y: float, height: float, radius: float) -> str:
    return f'''    <model name='cylinder_{index:04d}'>
      <static>1</static>
      <pose>{x:.6f} {y:.6f} {height / 2.0:.6f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry><cylinder><radius>{radius:.6f}</radius><length>{height:.6f}</length></cylinder></geometry></collision>
        <visual name='visual'>
          <geometry><cylinder><radius>{radius:.6f}</radius><length>{height:.6f}</length></cylinder></geometry>
          <material><ambient>0.447 0.624 0.812 1</ambient><diffuse>0.447 0.624 0.812 1</diffuse></material>
        </visual>
      </link>
    </model>'''


def render_world(args: argparse.Namespace, cylinders: List[Tuple[float, float, float]]) -> str:
    ground_size = max(args.size_x, args.size_y) + 2.0 * args.wall_thickness + 10.0
    half_x, half_y = args.size_x / 2.0, args.size_y / 2.0
    thickness = args.wall_thickness
    walls = [
        box_model("wall_north", 0.0, half_y + thickness / 2.0, args.size_x + 2.0 * thickness, thickness, args.wall_height),
        box_model("wall_south", 0.0, -half_y - thickness / 2.0, args.size_x + 2.0 * thickness, thickness, args.wall_height),
        box_model("wall_east", half_x + thickness / 2.0, 0.0, thickness, args.size_y, args.wall_height),
        box_model("wall_west", -half_x - thickness / 2.0, 0.0, thickness, args.size_y, args.wall_height),
    ]
    cylinder_xml = [cylinder_model(index, *cylinder, args.cylinder_radius)
                    for index, cylinder in enumerate(cylinders)]
    return f'''<?xml version="1.0" ?>
<sdf version='1.6'>
  <world name={quoteattr(args.world_name)}>
    <gravity>0 0 -9.8</gravity>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>1</shadows></scene>
    <light name='sun' type='directional'><cast_shadows>1</cast_shadows><direction>-0.5 0.1 -0.9</direction><diffuse>0.8 0.8 0.8 1</diffuse></light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>{ground_size:.6f} {ground_size:.6f}</size></plane></geometry></collision>
        <visual name='visual'><geometry><plane><normal>0 0 1</normal><size>{ground_size:.6f} {ground_size:.6f}</size></plane></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>
      </link>
    </model>
{chr(10).join(walls)}
{chr(10).join(cylinder_xml)}
  </world>
</sdf>
'''


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    try:
        cylinders = place_cylinders(args, rng)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_world(args, cylinders), encoding="utf-8")
    print(f"generated {output}: {len(cylinders)} cylinders, "
          f"{args.size_x:g} x {args.size_y:g} m, seed={args.seed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
