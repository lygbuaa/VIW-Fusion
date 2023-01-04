import time
from datetime import datetime
import rospy
from std_msgs.msg import String

counter = 0
rospy.init_node("loop_test", anonymous=True)
pub_heartbeat = rospy.Publisher(name="loop_test_hearbeat", data_class=String, queue_size=10)
while not rospy.is_shutdown():
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d-%H-%M-%S")
    pub_heartbeat.publish(dt_string)
    rospy.loginfo(dt_string)
    time.sleep(1.0)