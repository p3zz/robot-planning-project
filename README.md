### Build
From the root dir
```bash
colcon build --packages-select cpp_pubsub
```

### Run
Make the executable reachable:
```bash
. install/setup.bash
```

```bash
ros2 run cpp_pubsub publisher
```
## Collisions

### segment/segment
![segment_segmen_intersection](https://user-images.githubusercontent.com/84152495/204867692-210016da-d72c-479b-b5e6-1ad0e74572c4.png)

### segment/circle
![segment_circle_intersection](https://user-images.githubusercontent.com/84152495/204867856-3701396b-a975-4355-9c73-e82d65152227.png)

### segment/arc
![arc_segment_intersection](https://user-images.githubusercontent.com/84152495/204867933-0e69153b-77e3-4d62-bc3d-b66c5063b831.png)
