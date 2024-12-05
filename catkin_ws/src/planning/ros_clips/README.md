# ros_clips

ROS interface for the C Language Integrated Production System or [CLIPS](https://clipsrules.net/)

This repo uses [clipspy](https://github.com/noxdafox/clipspy/) for CLIPS on Python 3.

## Setup

1. Run ```pip install -r requirements.txt```


## ROS-CLIPS Bridge

### Structure

**Topics**


1. 
```
/ros_clips/clipspy_reset
```

of type `std_msgs/Empty`


2. 
```
/ros_clips/clipspy_clear
```

of type `std_msgs/Empty`


3. 
```
/ros_clips/clipspy_load_file
```

of type `std_msgs/String`


4. 
```
/ros_clips/clipspy_build_rule
```

of type `std_msgs/String`


5. 
```
/ros_clips/clipspy_build_template
```

of type `std_msgs/String`


6. 
```
/ros_clips/clipspy_build_facts
```

of type `std_msgs/String`


7. 
```
/ros_clips/clipspy_load_fact
```

of type `std_msgs/String`


8. 
```
/ros_clips/clipspy_retract_fact
```

of type `std_msgs/Int32`


**Services**

1. 
```
/ros_clips/run_planning
```

of type `ros_clips/RunPlanning` 


where RunPlanning.srv:

```
int64 steps
---
StringArray plan
```

with StringArray.msg:

```
string[]  data
```

2. 

```
/ros_clips/get_facts
```

of type `ros_clips/GetPrintMessage` 

and

```
/ros_clips/get_rules
```

of type `ros_clips/GetPrintMessage` 


where GetPrintMessage.srv:

```
---
StringArray data
```

### Initialization

Start the ros_clips in a different terminal:

```
rosrun ros_clips ros_clipspy_bridge.py
```

### Usage

**Cubes example**

First, load the clp file in your *PATH_TO_ros_clips* folder:

```
rostopic pub /ros_clips/clipspy_load_file std_msgs/String "data: '~<PATH_TO_ros_clips>/data/clp/cubes_stacks_rules.clp'" 
```

Then, check the current rules:

```
rosservice call /ros_clips/get_rules "{}"
```

and facts:

```
rosservice call /ros_clips/get_facts "{}"
```

You can add some new facts as follows:

```
rostopic pub /ros_clips/clipspy_load_fact std_msgs/String "data: '(get-initial-state stacks 2)'"
rostopic pub /ros_clips/clipspy_load_fact std_msgs/String "data: '(stack A B C)'"
rostopic pub /ros_clips/clipspy_load_fact std_msgs/String "data: '(stack D E F)'"
```

You can also add a new goal from a template. First, check the templates:

```
rosservice call /ros_clips/get_templates "{}"
```

Then, add the new goal as a fact:

```
rostopic pub /ros_clips/clipspy_load_fact std_msgs/String "data: '(goal (move F)(on-top-of A))'"
```

Finally, you can get a plan one step a at a time:

```
rosservice call /ros_clips/run_planning "steps: 1"
```

or all at once:

```
rosservice call /ros_clips/run_planning "steps: 0"
```

You can check the current facts, after executing the full plan:

```
rosservice call /ros_clips/get_facts "{}"
```

add a new goal:

```
rostopic pub /ros_clips/clipspy_load_fact std_msgs/String "data: '(goal (move A)(on-top-of F))'"
```

get the new plan:

```
rosservice call /ros_clips/run_planning "steps: 0"
```

and check the latest facts:

```
rosservice call /ros_clips/get_facts "{}"
```

