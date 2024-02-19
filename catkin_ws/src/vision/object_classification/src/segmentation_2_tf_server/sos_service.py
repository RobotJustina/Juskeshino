#! /usr/bin/env python3
import rospy                                      # the main module for ROS-python programs
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
                                                  # ...Other types are available, and you can create
                                                  # custom types
def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    return TriggerResponse(
        success=True,
        message="Segmentation requested"
    )

rospy.init_node('sos_service')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/segment_2_tf', Trigger, trigger_response         # type, and callback
)
rospy.spin()   
