import tf
import rospy

rospy.init_node('test') 
listener = tf.TransformListener()
listener.waitForTransform('/world', '/beetle1/root', rospy.Time(0), rospy.Duration(4.0))
listener.lookupTransform('/world', '/beetle1/root', rospy.Time(0))