import rospy
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
q = quaternion_from_euler(0,0, 3.134314)
print q