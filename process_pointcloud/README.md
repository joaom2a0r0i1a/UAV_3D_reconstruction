# Pointcloud Processing for Multi-Robot Mapping

This package processes incoming pointclouds from other UAVs in multi-robot exploration scenarios. It uses the localization of all drones to filter out points that correspond to the positions of other robots, preventing them from being mapped as part of the environment.

Adapted from the original implementation by Andreas Bircher and the Autonomous Systems Lab presented [here](https://github.com/ethz-asl/nbvplanner).
