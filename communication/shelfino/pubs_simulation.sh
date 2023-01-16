ros2 topic pub --once gate_position geometry_msgs/msg/PoseArray '{header:{stamp:{sec: 0, nanosec: 0}}, poses:[{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: -10, y: -5, z: 0}}, {orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 10, y: 5, z: 0}}]}' &&
ros2 topic pub --once map_borders geometry_msgs/msg/Polygon '{points:[{x: -10, y: -10, z: 0}, {x: -10, y: 10, z: 0}, {x: 10, y: 10, z: 0}, {x: 10, y: -10, z: 0}]}' \
&& ros2 topic pub --once obstacles custom_msgs/msg/ObstacleArrayMsg '{header:{stamp:{sec: 0, nanosec: 0}}, obstacles: [{header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 3, y: 1, z: 0}, {x: 3, y: 2, z: 0}, {x: 4, y: 2, z: 0}, {x: 4, y: 1, z: 0}]}, radius: 0}, {header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 8, y: 6, z: 0}, {x: 8, y: 7, z: 0}, {x: 9, y: 7, z: 0}, {x: 9, y: 6, z: 0}]}, radius: 0}]}' \
&& ros2 topic pub --once pursuer_pose geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 0, y: 0, z: 0}}' \
&& ros2 topic pub --once evader_pose geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 0, y: 1, z: 0}}' \