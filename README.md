# ROS 2 RRT* Global Planner for Autonomous Delivery

C++ implementation of an **RRT\***-based global path planner for autonomous delivery, originally developed for my master’s thesis.  
Built with **ROS 2 Jazzy**, **OpenCV**, and **FLANN KD-tree**, and visualized in **RViz**.

**Live demo:**

![Animated RRT*KD planning on an occupancy grid](media/rrt_star_kd_demo.gif)

*RRT*KD planning on an occupancy grid: exploration tree in blue, final optimized path in green.*

---

## What this repo demonstrates

- Real-time sampling-based path planning in 2D occupancy grids  
- **RRT\*** algorithm with KD-tree nearest-neighbor and radius search (FLANN)  
- Collision checking using a conservative rectangular vehicle footprint  
- ROS 2 node exposing planning as a service and publishing `nav_msgs/Path` + markers  
- End-to-end integration: map loading → planning → RViz visualization  

This is the core global planning component from my thesis on **autonomous last‑mile delivery**.

---

## Quick start

```bash
# From the repo root
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Launch planner with RViz
ros2 launch rrt_node rrt_star_kd_demo.launch.py
```

In RViz:

- Set **Fixed Frame** to `map`
- Add:
  - `/rrt_star_kd/path` (type: `Path`)
  - `/rrt_star_kd/markers` (type: `Marker` / `MarkerArray`)
- Press **Z** to zoom to fit

Call the planning service:

```bash
ros2 service call /rrt_star_kd/plan std_srvs/srv/Trigger {}
```

The planner computes a collision-free path and publishes it to RViz with:
- Start marker (green)
- Goal marker (red)
- RRT* exploration tree (blue)
- Final optimized path (green)

---

## Key design decisions

- **RRT\***  
  Chosen for asymptotic optimality and robustness in cluttered environments.  
  The implementation incrementally builds a tree of feasible configurations and rewires to improve path cost.

- **KD-tree (FLANN)**  
  Used for efficient nearest-neighbor and radius queries during sampling and rewiring, enabling scalable performance on larger maps.

- **Collision checking**  
  Conservative rectangular footprint with margin, approximating a small delivery vehicle.  
  Checks are performed in world coordinates against the occupancy grid.

- **ROS 2 integration**  
  Planner exposed as a synchronous service (`/rrt_star_kd/plan`).  
  Outputs standard `nav_msgs/Path` and visualization markers for easy integration with other navigation components.

---

## Thesis context

This planner was part of a larger system for **autonomous delivery**, which included:

- Occupancy-grid mapping  
- Global path planning (this component)  
- Trajectory tracking (MPC)  
- Vehicle-to-user interaction (QR-based handover)  
- High-level behavioral planning (finite-state machine)

Full thesis (M.Sc., Politecnico di Milano):  
**“Integrated Autonomous Delivery System: Enhancing Vehicle-to-User Interaction and Dynamic Path Planning”**

- [Thesis PDF](docs/Thesis.pdf)

---

## Repository structure

- `src/rrt_node/`
  - `src/rrt_star_kd_ros_node.cpp` – main RRT*KD planner and ROS 2 node  
  - `launch/rrt_star_kd_demo.launch.py` – demo launch file with RViz  
  - `launch/rrt_star_kd.rviz` – RViz configuration  
  - `maps/` – example occupancy-grid maps (PGM + YAML)  
- `media/`
  - `rrt_star_kd_demo.gif` – animated planning demo  

---

## Contact

I’m open to opportunities in **autonomy**, **path planning**, and **ROS 2 robotics software**.

- Email: <jn.rizkallah@gmail.com>  
- LinkedIn: [linkedin.com/in/johnmrizkallah](https://www.linkedin.com/in/johnmrizkallah)  
- GitHub: [github.com/johnrizkallah/ros2-autonomous-delivery-path-planning](https://github.com/johnrizkallah/ros2-autonomous-delivery-path-planning)
