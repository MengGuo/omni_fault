- Initialize robot model in robot_model_def.py
- Start robot node via 
```
python omni_planner.py Y1
```
- Manually send confirmation for robot next activity:
```
rostopic pub -1 activity_done_Y1 omni_fault/confirmation -- '0' 'goto' '1'
rostopic pub -1 activity_done_Y1 omni_fault/confirmation -- '1' 'y1act' '1'
```
- Manually send operation mode update to robot via
```
rostopic pub -1  status_Y1 omni_fault/status -- 'type-II'
```

- Manually send request to other robot via
```
rostopic pub -1  collab_request omni_fault/request -- 'Y2' '<> y1r1'
```