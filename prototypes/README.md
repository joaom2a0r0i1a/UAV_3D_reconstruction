# Algorithm Prototypes

Standalone 2D/3D sampling-based-planner prototypes used to design and study the algorithms before the
C++ implementation. **Not used by any ROS package** — production planning lives in
`core/rrt_construction` (`rrt_star_kd.cpp`, `kino_rrt_star_kd.cpp`, `kd_tree.cpp` + nanoflann).
This directory is intentionally NOT a catkin package (no `package.xml`) so rospack/roslaunch never scan it.

Run any file directly: `python3 <file>.py` (needs numpy + matplotlib; `prm_2d.py` also scipy).

## rrt/
| file | demo |
|---|---|
| `rrt.py` | basic 3D RRT |
| `rrt_animated.py` | RRT with live matplotlib animation |
| `kino_rrt.py` | kinodynamic RRT (velocity-state steering) |

## rrt_star/
| file | demo |
|---|---|
| `rrt_star.py` | 3D RRT* |
| `rrt_star_animated.py` | RRT* with live animation |
| `rrt_star_2d.py` / `rrt_star_2d_animated.py` | 2D variants |
| `rrt_star_yaw.py` / `rrt_star_2d_yaw.py` | RRT* with yaw in the state |
| `rrt_star_rewired.py` | isolated rewire-step study |
| `prm_2d.py` | 2D probabilistic roadmap (scipy KD-tree) |
