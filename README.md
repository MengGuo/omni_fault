# Motion and Task Planning for Omni-Directional Ground Vehicles

## Single vehicle test
The robot is assigned a task to **infinitely often** load object *a* (either at region `la1` with orientation `west` or at region `la2` with orientation `east`), then unload it (either at region `ua1` with orientation `south` or at region `ua2` with orientation `north`) and visit its base `h1`, and which is specified as the LTL formula:
```python
one_la = '(la1 && w) || (la2 && e)'
one_ua = '(ua1 && s)|| (ua2 && n)'
Y1_task = '[] <> ((%s &&ts loada) && <> (%s && unloada)) && [] <> (h1 && e)' %(one_la, one_ua)
```

- Initialize robot model in robot_model_def.py
 
- Start robot node via 
```python
python omni_planner.py Y1
```

- Manually send confirmation for robot next activity:
```
rostopic pub -1 activity_done_Y1 omni_fault/confirmation -- '0' 'goto' '1'
rostopic pub -1 activity_done_Y1 omni_fault/confirmation -- '1' 'la1' '1'
```

- Manually send operation mode update (*faults*) to robot via
```
rostopic pub -1  status_Y1 omni_fault/status -- 'type-II'
```
  Then once the robot the current motion or action is confirmed (you could do it manually like above), you will see that the discrete plan has been updated due to the cost update in the underlying transition system. 

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
  Then you will see the request and reply messages exchanged among the robots to negotiate which robot will share the task for `Y2`. Once the other robots accomplish their current motion or action (you could do it manually), e.g.,
```
rostopic pub -1 activity_done_Y3 omni_fault/confirmation -- '0' 'goto' '1'
rostopic pub -1 activity_done_Y4 omni_fault/confirmation -- '0' 'goto' '1'
```
  then the task of `Y2` will be assigned to one robot, depending on the reply messages. 


## Scenario description

Four robots (`Y1`, `Y2`, `Y3`, `Y4`) start from two home bases (`h1`, `h2`): 

- two of them are assigned to loard object `a` from any of the loading stations (`la1`, `la2`) with a certain orientation (`w`, `e`, `n`, `s`), and then unload `a` to the unloading station (`ua`) with a certain orientation, and then return to its base to charge. This procedure is repeated infinitely often.

- the other two are assigned to loard object `b` from any of the loading stations (`lb1`, `lb2`) with a certain orientation (`w`, `e`, `n`, `s`), and then unload `b` to the unloading station (`ub`) with a certain orientation, and then return to its base to charge. This procedure is repeated infinitely often.

During this process, robot `Y1` will experience `Type-I` failure and robot `Y4` will experience `Type-II` failure. They both recover automatically by adapting its lower-level controller and high-level plan. Then robot `Y3` will stop due to `Type-III` failure and its task will be re-assigned to another robot, based on online robot-to-robot communication. 
