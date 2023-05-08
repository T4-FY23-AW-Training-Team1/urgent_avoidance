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

Currently, when the service /provoke_avoidance is called.

The goal of the vehicle will be changed to the position of one meter ahead of the vehicle.

**But should be revised to set the goal to a nearby road shoulder in the future.**