# ROS 2 Autonomous Delivery Path Planning

ROS 2 modernization of the navigation components from my master's thesis:
**Integrated Autonomous Delivery System: Enhancing Vehicle-to-User Interaction and Dynamic Path Planning**.

## Current milestone

A standalone C++ implementation of **RRT*KD** that:

- Loads an occupancy-grid map from YAML and PGM files
- Uses OpenCV for map processing
- Uses FLANN KD-tree nearest-neighbor and radius searches
- Performs collision checking with a conservative vehicle footprint
- Produces a collision-free path in world coordinates
- Exports planning output as a text path and JSON data file

## Thesis context

The original system combined:

- Occupancy-grid mapping
- RRT*KD global path planning
- Model Predictive Control trajectory tracking
- QR-based vehicle-to-user interaction
- Finite-state behavioral planning

## Status

Work in progress: converting the standalone RRT*KD implementation into a ROS 2 node that will publish `nav_msgs/Path` for RViz visualization.
