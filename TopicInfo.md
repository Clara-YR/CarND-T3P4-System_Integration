# Topic information
![](Readme_Images/RosTopic/rostopic_list.png)

<a name="table"></a>

|Topics|msg Type|Publishers|Subscribers|
|:----:|:------:|:--------:|:---------:|
|[`/base_waypoints`](#bw)|[`Lane`](#L)|	`Waypoint Loader`|`tl_detector`<br />`waypoint_updater`|
|[`/current_pose`](#cp)|[`PoseStamped`](#PS)|
|[`/current_velocity`](#cv)|[`TwistStamped`](#TS)|
|[`/final_waypoints`](#fw)|[`Lane`](#L)|
|[`/image_color`](#ic)|[`Image`](#T)|
|[`/tf`](#tf)|[`tfMessage`](#M)|
|[`/traffic_waypoint`](#tw)|[`Int32`](#32)|
|[`/twist_cmd`](#tc)|[`TwistStamped`](#TS)|
|[`/vehicle/dbw_enabled`](#de)|[`Bool`](#B)|
|[`/vehicle/brake_cmd`](#bc)|[`BrakeCmd`](BC)|
|[`/vehicle/steering_cmd`](#sc)|[`SteeringCmd`](#SC)|
|[`/vehicle/throttle_cmd`](#thc)|[`ThrottleCmd`](#TC)|
|[`/vehicle/traffic_lights`](#tl)|[`TrafficLightArray`](#TLA)|

![](Readme_Images/code_structure.png)

<a name="bw"></a>
## `/base_waypoints`
- msg Type: [[`Lane`](#L)
![](Readme_Images/RosTopic/Topic_base_wp.png)

[Back to Table](#table)
<a name="cp"></a>
## `/current_pose`
- msg Type: [`PoseStamped`](#PS)

![](Readme_Images/RosTopic/Topic_current_pose.png)

[Back to Table](#table)
<a name="cv"></a>
## `/current_velocity`
- msg Type: [`TwistStamped`](#TS)

![](Readme_Images/RosTopic/Topic_current_vel.png)

[Back to Table](#table)
<a name="fw"></a>
## `/final_waypoints`
- msg Type: [`Lane`](#L)

![](Readme_Images/RosTopic/Topic_final_wp.png)

[Back to Table](#table)
<a name="ic"></a>
## `/image_color`
- msg Type: [`Image`](#I)

![](Readme_Images/RosTopic/Topic_image_color.png)


`/rosout`

`/rosout_agg`

[Back to Table](#table)
<a name="tf"></a>
## `/tf`
- msg Type: [`tfMessage`](#M)

![](Readme_Images/RosTopic/Topic_tf.png)

[Back to Table](#table)
<a name="tw"></a>
## `/traffic_waypoints`
- msg Type: [`Int32`](#32)

![](Readme_Images/RosTopic/Topic_traffic_wp.png)

[Back to Table](#table)
<a name="tc"></a>
## `/twist_cmd`
- msg Type: [`TwistStamped`](#TS)

![](Readme_Images/RosTopic/Topic_twist_cmd.png)

[Back to Table](#table)
<a name="de"></a>
## `/vehicle/dbw_enabled`
- msg Type: [`Bool`](#B)

![](Readme_Images/RosTopic/Topic_dbw_enabled.png)

[Back to Table](#table)
<a name="thc"></a>
## `/vehicle/throttle_cmd`
- msg Type: [`ThrottleCmd`](#TC)

![](Readme_Images/RosTopic/Topic_throttle_cmd.png)

[Back to Table](#table)
<a name="sc"></a>
## `/vehicle/steering_cmd`
- msg Type: [`SteeringCmd`](#SC)

![](Readme_Images/RosTopic/Topic_steering_cmd.png)

[Back to Table](#table)
<a name="bc"></a>
## `/vehicle/brake_cmd`
- msg Type: [`BrakeCmd`](#BC)

![](Readme_Images/RosTopic/Topic_brake_cmd.png)

[Back to Table](#table)
<a name="tl"></a>
## `/vehicle/traffic_lights`
- msg Type: [`TrafficLightArray`](#TLA)

![](Readme_Images/RosTopic/Topic_traffic_lights.png)

`/obstacle_waypoints`

# Message Structure

[Back to Table](#table)
<a name="L"></a>
## `Lane`				
![](Readme_Images/RosTopic/msg_Lane.png)

[Back to Table](#table)
<a name="PS"></a>
## `PoseStamped`
![](Readme_Images/RosTopic/msg_PoseStamped.png)

[Back to Table](#table)
<a name="TS"></a>
## `TwistStamped`
![](Readme_Images/RosTopic/msg_TwistStamped.png)

[Back to Table](#table)
<a name="I"></a>
## `Image`
![](Readme_Images/RosTopic/msg_Image.png)

[Back to Table](#table)
<a name="M"></a>
## `tfMessage`
![](Readme_Images/RosTopic/msg_tfMessage.png)

[Back to Table](#table)
<a name="32"></a>
## `Int32`
![](Readme_Images/RosTopic/msg_Int32.png)

[Back to Table](#table)
<a name="B"></a>
## `Bool`
![](Readme_Images/RosTopic/msg_Bool.png)

[Back to Table](#table)
<a name="TC"></a>
## `ThrottleCmd`
![](Readme_Images/RosTopic/msg_ThrottleCmd.png)

[Back to Table](#table)
<a name="SC"></a>
## `SteeringCmd`
![](Readme_Images/RosTopic/msg_SteeringCmd.png)

[Back to Table](#table)
<a name="BC"></a>
## `BrakeCmd`
![](Readme_Images/RosTopic/msg_BrakeCmd.png)

[Back to Table](#table)
<a name="TLA"></a>
## `TrafficLightArray`
![](Readme_Images/RosTopic/msg_TrafficLightArray.png)

[Back to Table](#table)



