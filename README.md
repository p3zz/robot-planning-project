### Packages
- navigation: subscribers, publishers and clients that communicate with Shelfino
- planner: planning logic
- dubins: dubin logic
- map: roadmap logic
- shapes: basic shapes (polygon, segment, point) with collisions checks
- utils
- obstacles_msgs, graph_msgs: custom messages

### Testing
3 test suites:
- dubins: dubin curve - segment, dubin curve - polygon collision checks
- shapes: general collsion checks
- map: roadmap build's performance

To run a test:
```bash
colcon test --packages-select package_name --event-handler=console_direct+
```

### Simulation
Step-by-step simulation using both pursuer and evader.
To run the simulation:
```bash
ros2 run main_tester main_tester
```
Once the simulation is completed, 2 files will be available: 
- map.json: contains the configuration of the whole roadmap (map borders, nodes, links, obstacles and exits)
- moves.json: contains the sequence of moves of both pursuer and evader

To visualize only the map:
```bash
cd disegnino
python3 main.py path/to/map  
```

To visualize the whole race:
```bash
cd disegnino
python3 main.py path/to/map path/to/moves  
```

### Build
From the root dir
```bash
colcon build
```

### Run
Make the executable reachable:
```bash
. install/setup.bash
```

Run the communication stack:
```
ros2 run navigation navigation
```
