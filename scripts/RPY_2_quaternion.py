import rospy
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
import numpy as np
def normalize(array):
    quat = np.array(array)
    return quat / np.sqrt(np.dot(quat, quat))


q = quaternion_from_euler(0,0, -2.35619449)
print q
print 'before normal'
q = normalize(q)
print q
print 'after normal'