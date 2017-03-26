# Motion and Task Planning for Omni-Directional Ground Vehicles

## Single vehicle test
The robot is assigned a task to __ **infinitely often** load object *a* (either at region `la1` with orientation `west` or at region `la2` with orientation `east`), then unload it (either at region `ua1` with orientation `south` or at region `ua2` with orientation `north`) and visit its base `h1`, and which is specified as the LTL formula:
```python
one_la = '(la1 && w) || (la2 && e)'
one_ua = '(ua1 && s)|| (ua2 && n)'
Y1_task = '[] <> ((%s && loada) && <> (%s && unloada)) && [] <> (h1 && e)' %(one_la, one_ua)
```

- Initialize robot model in robot_model_def.py
 
- Start robot node via 
```python
python omni_planner.py Y1
```

- Manually send confirmation for robot next activity:
```
rostopic pub -1 activity_done_Y1 omni_fault/confirmation -- '0' 'goto' '1'
rostopic pub -1 activity_done_Y1 omni_fault/confirmation -- '1' 'y1act' '1'
```

- Manually send operation mode update (*faults*) to robot via
```
rostopic pub -1  status_Y1 omni_fault/status -- 'type-II'
```
  Then you will see that the discrete plan has been updated due to the cost update in the underlying transition system. 

## Collaboration between multiple vehicles

There are four robots in

- Start all four robots via (in different terminals) 
```python
python omni_planner.py Y1
python omni_planner.py Y2
python omni_planner.py Y3
python omni_planner.py Y4
```

- Manually set robot `Y2` stops due to `type-III` fault
```
rostopic pub -1  status_Y2 omni_fault/status -- 'type-III'
```
  Then you will see the request and reply messages exchanged among the robots to negotiate which robot will share the task for `Y2`.