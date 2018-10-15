# Topic in `/darknet_ros`
![](Readme_Images/Darknet/Darknet_Topic.png)

<a name="table"></a>

|Topics|msg Type|Publishers|Subscribers|
|:----:|:------:|:--------:|:---------:|
|[/bounding_boxes](#bb)|[`BoundingBoxes`](#BB)|`/darknet_ros`|`/tl_detector`|
|[/check\_for_objects/result](#r)|[`CheckForObjectsActionResult`](#R)|`/darknet_ros`|None|
|[/check\_for_objects/cancel](#c)|[`CheckForObjectsActionCancel`](#C)|None|`/darknet_ros`|
|[/check\_for_objects/feedback](#f)|[`CheckForObjectsActionFeedback`](#F)|`/darknet_ros`|None|
|[/check\_for_objects/goal](#g)|[`CheckForObjectsActionGoal`](#G)|None|`/darknet_ros`|
|[/check\_for_objects/status](#s)|[`CheckForObjectsActionStatus`](#S)|`/darknet_ros`|None|



![](Readme_Images/code_structure.png)

<a name="bb"></a>
## `/bounding_boxes`
- msg Type: [`BoundingBoxes`](#BB)

![](Readme_Images/Darknet/Topic_bounding_boxes.png)

[Back to Table](#table)

<a name="r"></a>
## `/check_for_objects/result`
- msg Type: [`CheckForObjectsActionResult`](#R)

![](Readme_Images/Darknet/Topic_result.png)

[Back to Table](#table)

<a name="c"></a>
## `/check_for_objects/cancel`
- msg Type: [`actionlib_msgs/GoalID`](#C)

![](Readme_Images/Darknet/Topic_cancel.png)

[Back to Table](#table)

<a name="f"></a>
## `/check_for_objects/feedback`
- msg Type: [`CheckForObjectsActionFeedback`](#F)

![](Readme_Images/Darknet/Topic_feedback.png)

[Back to Table](#table)

<a name="g"></a>
## `/check_for_objects/goal`
- msg Type: [`CheckForObjectsActionGoal`](#G)

![](Readme_Images/Darknet/Topic_goal.png)

[Back to Table](#table)

<a name="s"></a>
## `/check_for_objects/status`
- msg Type: [`actionlib_msgs/GoalStatusArray`](#S)

![](Readme_Images/Darknet/Topic_status.png)

[Back to Table](#table)

# Message Structure in `darknet_ros_msgs`

<a name="BB"></a>
## `BoundingBoxes`

![](Readme_Images/Darknet/msg_BoundingBoxes.png)				
![](Readme_Images/Darknet/BoundingBoxes_Echo.png)

[Back to Table](#table)

<a name="R"></a>
## `CheckForObjectsActionResult`

![](Readme_Images/Darknet/msg_Result.png)

[Back to Table](#table)

<a name="C"></a>
## `CheckForObjectsActionCancel`

![](Readme_Images/Darknet/msg_Cancel.png)

[Back to Table](#table)

<a name="F"></a>
## `CheckForObjectsActionFeedback`

![](Readme_Images/Darknet/msg_Feedback.png)

[Back to Table](#table)

<a name="G"></a>
## `CheckForObjectsActionGoal`

![](Readme_Images/Darknet/msg_Goal.png)

[Back to Table](#table)

<a name="S"></a>
## `CheckForObjectsActionStatus`

![](Readme_Images/Darknet/msg_Status.png)

[Back to Table](#table)