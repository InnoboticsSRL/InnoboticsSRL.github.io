# ibt_ros2_interfaces

## Messages

### `ibt_ros2_interfaces/msg/MoveReq.msg`

Move request message for a robotic ARM

- `int32 move_type`: Possible type of movement
- `Waypoint[] waypoints`: List of waypoints to move through
- `float64 angle`
- `float64 velocity`
- `float64 acceleration`
- `float64 rotational_velocity`
- `float64 rotational_acceleration`

### `ibt_ros2_interfaces/msg/PoseRPY.msg`

Eulerian pose

- `float64 x`
- `float64 y`
- `float64 z`
- `float64 roll`
- `float64 pitch`
- `float64 yaw`

### `ibt_ros2_interfaces/msg/Waypoint.msg`

Waypoint message for defining a target

- `float64[6] pose`: pose in Cartesian or joint space
- `float64 smoothing_factor`: waypoint smoothing factor in the range [0..1]
- `float64 next_segment_velocity_factor`: segment velocity factor in the range [0..1]

## Services

### `ibt_ros2_interfaces/srv/GetAttrAll.srv`

**Request**
- `uint16 clas`
- `uint8 instance`

**Response**
- `uint8 result_code`
- `uint16[] result`

### `ibt_ros2_interfaces/srv/SetAttrAll.srv`

**Request**
- `uint16 clas`
- `uint8 instance`
- `uint16[10] data`

**Response**
- `uint8 result_code`

### `ibt_ros2_interfaces/srv/SetOutput.srv`

**Request**
- `bool[] data`

**Response**
- `bool success`
- `string message`

## Actions

### `ibt_ros2_interfaces/action/MoveArm.action`

**Goal**
Request

- `std_msgs/Header header`
- `MoveReq[] requests`

**Result**
Result

- `int32 error_code`: error code
- `string error_str`: error message

**Feedback**
Feedback

- `string status`: status of the move
