## init

启动起飞服务
```
rosservice call /waypoint_runner_node/start_mission "{}"
```
## 控制器使用

解锁
- 如果有命名空间比如 /iris_0 :
```
rosservice call /iris_0/trajectory_executor/takeoff "{}"
```
- 无命名空间:
```
rosservice call /trajectory_executor/takeoff "{}"
```

发送航点飞行
- 有命名空间:
```
rostopic pub -1 /iris_0/trajectory_executor/goto geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```
- 无命名空间:
```
rostopic pub -1 /trajectory_executor/goto geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```
