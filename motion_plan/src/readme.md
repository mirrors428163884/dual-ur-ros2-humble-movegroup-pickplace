### 规划器速度控制
motion plan代码中的设定速度只是原本的大小，比如设置为0.9

joint_limits.yaml的
```yaml
    has_velocity_limits: true
    max_velocity: 0.10
    has_acceleration_limits: true
    max_acceleration: 0.5
```
又给关节速度上了倍率? $\;$ 0.9 $\small \times$ 0.1 = 0.09 为最终速度？

`dual_arm_moveit_config/config/pilz_cartesian_limits.yaml`参数不起作用？



1. 用来查看pilz的速度参数名称
- ros2 param list /move_group --filter .*cartesian_limits
---

2. 用来查看速度参数值
- ros2 param get /move_group robot_description_planning.cartesian_limits.max_trans_vel


---
demo.launch.py:
pilz_industrial_motion_planner_planning.yaml是规划器设置，如果没写这些'pilz_industrial_motion_planner': 
                'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner'就需要外部的文件, 这里和下面的重复了。这个yaml文件类似ompl_planning_pipeline_config，
类似于ompl_planning.yaml，他的参数写在了另一个文件pliz_cartesian_limits.yaml中，包括速度等参数