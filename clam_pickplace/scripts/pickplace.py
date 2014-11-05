import sys,Canvas rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped




class ClamPickPlace
    robot = RobotCommander()
    def __init__():
        self.scene = PlanningSceneInterface()
        self.size = (0.15, 0.1, 0.3)

    def add_part(self, stamp=None):
        if stamp is None:
            stamp = PoseStamped()
            stamp.pose.position.x = 0.6
            stamp.pose.position.y = -0.7
            stamp.pose.position.z = 0.5
        scene.add_box("part", stamp, self.size)
        self.last_sent = stamp

    def pick(self):
        rospy.sleep(2) # bug why sleep?
        robot().arm.pick("part")
        
if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    a = ClamPickPlace()
    a.add_part()
    a.pick()
    
    rospy.spin()
    roscpp_shutdown()
