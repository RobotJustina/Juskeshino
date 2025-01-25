#Define constants for: WAVING, POINTING_RIGHT, POINTING_LEFT,
uint16 NONE=0
uint16 WAVING=1
uint16 POINTING_RIGHT=2
uint16 POINTING_LEFT=3
uint16 HANDLING_RIGHT=4
uint16 HANDLING_LEFT=5
uint16 RECEIVING_LEFT=6
uint16 RECEIVING_RIGHT=7
uint16 SITTING=8
uint16 STANDING=9
uint16 LYING=10

uint16 id # One of the previous predefined values
geometry_msgs/PoseStamped coordinate #Relevant pose data depending on the gesture type
sensor_msgs/Image img  #Image with the gesture highlighted
