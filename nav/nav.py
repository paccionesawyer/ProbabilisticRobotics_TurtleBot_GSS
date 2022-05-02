
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
import actionlib
from geometry_msgs.msg import Point

class Navigation():
    def __init__(self, x, y):
        # Initialize node
        rospy.init_node('Navigation', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL + C") 
        rospy.on_shutdown(self.shutdown)

        # Set goal location
        self.goal_x = x
        self.goal_y = y
        self.goal_reached = False

        self.goal_reached = self.move_to_goal(self.goal_x, self.goal_y)

        # Determine outcome
        if self.goal_reached:
            rospy.loginfo("Success")
        else:
            rospy.loginfo("Didn't quite make it")

        while not rospy.is_shutdown():
            self.r.sleep()

    # Move towards given position in the map
    def move_to_goal(self, x_goal, y_goal):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for action server
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

        # Set goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move towards goal
        goal.target_pose.pose.position =  Point(x_goal,y_goal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        # Wait until finished
        ac.wait_for_result(rospy.Duration(60))

        # Determine whether successful
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False        
    
    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        rospy.sleep(1)

if __name__ == '__main__':
    # Goal location in global map coordinates
    if len(sys.argv) != 3:
        x = -2.8 
        y = 2.2
    else:
        x = sys.argv[1]
        y = sys.argv[2]   

    try:
        Navigation(x, y)
        rospy.loginfo("Goal reached successfully.")
        rospy.spin()
    except:
        rospy.loginfo("Navigation node terminated.")