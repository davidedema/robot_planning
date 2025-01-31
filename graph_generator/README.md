# Instruction for running the planners

Run at first the simulation with

```
ros2 launch projects evacuation.launch.py use_rviz:=false generate_new_map_config:=false gen_map_params_file:=src/map_pkg/config/demos/evacuation/evacuation<1-3>.yaml 
``` 

or if you want a random map
```
ros2 launch projects evacuation.launch.py use_rviz:=false
``` 


## Sampling based

Run planner + nav2 client
```
ros2 launch graph_generator sampling.launch.py
```

## Combinatorial based

Run planner + nav2 client
```
ros2 launch graph_generator combinatorial.launch.py
```