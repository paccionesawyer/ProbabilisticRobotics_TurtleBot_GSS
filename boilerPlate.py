import sys
import rospy
from std_msgs.msg import String

sub_topic = ""
pub_topic = ""
service   = ""

sub_msg_type = None
pub_msg_type = None
service_msg_type = None

class BoilerPlate:
    '''
    Boilerplate code for subscribing and publshing in ROS
    '''

    def __init__(self, namespace: str = ""):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'boiler_plate'
        '''
        self.pub            = rospy.Publisher(pub_topic, pub_msg_type, queue_size=10)
        self.subscription   = rospy.Subscriber(sub_topic, sub_msg_type, self.listener_callback)
        self.service        = rospy.Service(service, service_msg_type, self.callback_reset_counter)

    def listener_callback(self, data):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) get's a message this function is 
        'called back' to and ran.
        '''
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def service_callback(self, req):
        '''
        Purpose
        -------
        Whenever our service get's a message this function will run
        '''

def main(args=None):
    rospy.init_node('boiler_plate')

    boiler_plate = BoilerPlate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")


if __name__ == '__main__':
    main()