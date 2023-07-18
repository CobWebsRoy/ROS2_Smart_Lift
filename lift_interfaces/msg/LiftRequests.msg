#lift ID
int64 lift_id

#robot ID
int64 robot_id

#current floor of robot
int64 robot_current_floor

#destination floor of robot
int64 robot_destination_floor

#current status of robot
int64 robot_status

# 0 = outside lift
# 1 = RELEASE LIFT (not using lift anymore)
# 2 = inside lift

# EXAMPLE: to publish a lift request:
# ros2 topic pub -1 /lift_requests lift_interfaces/LiftRequests "{lift_id: 1, robot_id: 1, robot_current_floor: 2, robot_destination_floor: 3, robot_status: 0}"
