# urgent_avoidance
This repository includes packages which are useful for planning of avoiding emergency vehicles

## goal_update_srvs
This is a package that contains a service type `FindRoadShoulderGoal.srv`.

**The name of this service should be refactored in the future.**

The format of `FindRoadShoulderGoal.srv` looks like this.

```
geometry_msgs/PoseStamped current_pose
---
geometry_msgs/PoseStamped goal_pose
```

Given the current pose, the altered goal pose should be returned.

## current_pose_talker

Given a topic /tf, this node converts it to a `geometry_msgs/PoseStamped` type message topic.

## urgent_avoidance

Given a specific trigger, urgent_avoidance node let the vehicle to pull over to a nearby road shoulder.

### Triggering avoidance event

There are two ways to trigger this.
1. Call the `/provoke_avoidance` service. (No request required)
2. Considering avoidance of emergency vehicles, having a specific sound event will also trigger pull overing.
    - This node is subscribing a `SoundSourceDirection` type topic which is defined in `acoustics_msgs` package.
    - If the sound source direction is between `rear_angle_range_min` to `rear_angle_range_max`, and that sound source direction has been kept longer than `direction_duration_threshold`, this node will trigger the avoidance.

### Searching road shoulder

This node will search a road shoulder nearest to the planned trajectory before avoidance, and set the goal pose at the centerline of it.
However, if the goal is too close to the vehicle, the goal planner cannot create a feasible goal, so the searching process will start searching from `search_starting_distance` ahead of the vehicle. This distance is not a straight line distance, but a distance along the planned trajectory.

### Chaging paramters

The four paramters `rear_angle_range_min`, `rear_angle_range_max`, `direction_duration_threshold`, and `search_starting_distance` can be set by updating `urgent_avoidance.param.yaml`.