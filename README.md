# Joint Potential-Vector Fields for Obstacle-Aware Legible Motion Planning

Official implementation of the paper **"Joint Potential-Vector Fields for Obstacle-Aware Legible Motion Planning"** by Huy Quyen Ngo and Aaron Steinfeld (Robotics Institute, Carnegie Mellon University), published at the *2024 33rd IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)*.

[![DOI](https://img.shields.io/badge/DOI-10.1109%2FRO--MAN60168.2024.10731443-blue)](https://doi.org/10.1109/RO-MAN60168.2024.10731443)

## Citation

If you use this code or build on this work, please cite the paper:

```bibtex
@inproceedings{ngo2024joint,
  title={Joint Potential-Vector Fields for Obstacle-Aware Legible Motion Planning},
  author={Ngo, Huy Quyen and Steinfeld, Aaron},
  booktitle={2024 33rd IEEE International Conference on Robot and Human Interactive Communication (ROMAN)},
  pages={1856--1863},
  year={2024},
  organization={IEEE}
}
```

## Acknowledgments

This work was supported by the Office of Naval Research award N00014-181-2503.

## Contact

- Huy Quyen Ngo — `huyquyen@andrew.cmu.edu` or `ngohuyquyen1997@gmail.com`
- Aaron Steinfeld — `steinfeld@cmu.edu`

Robotics Institute, Carnegie Mellon University, Pittsburgh, Pennsylvania, USA.


## Overview

This repository contains the simulation code for a 2D legible motion planner that combines **Potential Fields (PF)** for goal attraction/repulsion with **adaptive directional Vector Fields (VF)** for real-time obstacle avoidance. The method generates intent-expressive ("legible") robot motions toward one of multiple candidate goals, while smoothly avoiding obstacles that appear along the way — without sacrificing legibility.

In human-centered environments where a robot has multiple available goals (e.g., a robot server choosing between two tables), the shortest path is the most *predictable* but rarely the most *legible*. Our planner produces motions that:

- Communicate intent toward a specific real goal among multiple candidates.
- Avoid obstacles in real-time using circular vector fields whose direction is computed adaptively from the relative positions of the goal, obstacle, and agent's current heading.
- Replan smoothly when the goal switches mid-trajectory (e.g., the originally targeted hand or table becomes occupied).

The method is benchmarked against the legible motion planner of Dragan et al. (2013) and a traditional Potential Field planner on three scenarios: no-obstacle, with-obstacle, and goal-switching.

## Method Summary

The planner combines two field components:

1. **Potential Field for goals.** The real goal exerts an attractive force; other (decoy) goals exert repulsive forces. A `(d_goal)^n` decay term in the repulsive component ensures the real goal remains the global minimum of the field, mitigating the local-minimum problem of classical PF.
2. **Adaptive circular Vector Field for obstacles.** Each obstacle within its effective radius generates a circular force field. The direction (clockwise or counterclockwise) is determined by checking, at the moment of first contact, whether the goal and the obstacle lie on the same or opposite sides of the agent's heading line — an idea inspired by the BUG family of algorithms.

Refer to Section III of the paper for the full formulation.

## Repository Structure

```
.
├── implementation/   # Main simulation script (planner + plotting)
├── examples/         # Generated path and vector field visualizations
├── results/          # Comparison results on the AULC and TPL metrics
├── legacy/           # Earlier code and visualization files kept for history
└── README.md
```

The main script implements all components in one file, including:

- `add_goal`, `add_goal_decoy` — attractive and repulsive potential fields for goals.
- `circular_force_field` — adaptive directional vector field around obstacles.
- `obstacle_direction` — algorithm for selecting the rotation direction of the obstacle vector field based on relative goal/obstacle positions about the agent's heading.
- `get_path_from_field` — numerical integrator that produces the path by following the combined field from the start point.
- `new_path` — recomputes the path when the agent enters an obstacle's effective radius.
- `part_1`, `part_2` — top-level driver functions for the no-obstacle/with-obstacle case and the goal-switching case respectively.
- `plot_graph_g`, `plot_graph_g_switch` — visualization utilities for plotting goals, obstacles, start point, and trajectories.

## Requirements

- Python 3.8+
- NumPy
- SciPy
- Matplotlib
- Math

Install dependencies:

```bash
pip install numpy scipy matplotlib math
```

## Usage

Run the simulation with:

```bash
python novel_2d_legibility.py
```

The script will produce a Matplotlib figure showing:

- The real goal (blue) and other goal (red).
- Any configured obstacles (black).
- The start point (gray).
- The planned trajectory.

### Configuring Scenarios

Scenario configuration is done by editing the parameters at the top of `novel_2d_legibility.py`.

**No-obstacle case** (default):
```python
obstacles = []
```

**With-obstacle case:** uncomment one of the predefined obstacle lists, for example:
```python
obstacles = [[4.1, 2.0], [1.5, 4.0]]
```

**Goal-switching case:** swap the `goal` and `goal_decoy` positions and pass `flag=True` to `new_path` so that the planner triggers a replan at the switch point.

### Key Parameters

The parameters below correspond to those in Table I of the paper:

| Parameter | Variable | Default | Description |
|---|---|---|---|
| Start position | `start_point` | `[3.0, 0.0]` | Robot starting position |
| Real goal | `goal` | `[4.0, 6.0]` | Target goal location |
| Other goal | `goal_decoy` | `[2.0, 6.0]` | Decoy/distractor goal location |
| Attractive coefficient | `ka` | `1.0` | Strength of pull toward the real goal (`k_p` in paper) |
| Repulsive coefficient | `kp` | `0.2` | Strength of repulsion from the other goal (`k_n` in paper) |
| Vector field decay | `ko` | `0.8` | Decay coefficient `m` in the obstacle vector field |
| Other-goal decay | `n` | `5` | Decay coefficient `n` in the decoy repulsive field |
| Obstacle effective radius | `s_obs` | `10` | Range within which an obstacle's vector field is active |
| Obstacle radius | `r_obs` | `1.0` | Physical obstacle radius |
| Goal radius | `r_goal` | `2.5` | Convergence threshold for reaching a goal |
| Field strength | `field_strength` | `50` | Magnitude of the obstacle vector field |
| Integration step | `step_size` | `0.001` | Step size for path integration |

To exaggerate the legibility of the motion, decrease `ka` and increase `kp` (and tune `n` accordingly). To make obstacle avoidance more pronounced, increase `field_strength` and tune `ko`. See Section VI-A of the paper for further discussion of parameter tuning.

## Evaluation

The paper evaluates the planner using two metrics:

- **Area Under the Legibility Curve (AULC)** — integrates the legibility score (Dragan et al., 2013) over the trajectory, providing a path-wide measure of legibility rather than a single endpoint score. Higher is more legible.
- **Total Path Length (TPL)** — sum of the lengths of all path segments. Lower is more efficient.

Across the three scenarios, the proposed method produces paths that are:

- Of comparable legibility to the Dragan et al. legible baseline, but **shorter** (8.4%, 10.7%, and 18.6% shorter on the no-obstacle, with-obstacle, and goal-switch cases respectively).
- Significantly more legible than the traditional Potential Field planner (67.6%, 93.6%, and 43.5% more legible on the same three cases).

Refer to Table II and Figures 5–6 of the paper for the full numerical results.
