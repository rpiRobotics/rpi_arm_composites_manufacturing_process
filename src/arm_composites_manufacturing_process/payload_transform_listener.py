import general_robotics_toolbox as rox
from general_robotics_toolbox import ros_tf as rox_tf
from general_robotics_toolbox import ros_msg as rox_msg
from rpi_arm_composites_manufacturing_process.msg import PayloadArray
from geometry_msgs.msg import TransformStamped, Vector3

import numpy as np
import rospy

from tf2_ros import TransformException, ConnectivityException, LookupException, ExtrapolationException

def _point_to_vector3(p):
    return Vector3(p.x, p.y, p.z)


class PayloadTransformListener(rox_tf.TransformListener):
    def __init__(self):
        super(PayloadTransformListener,self).__init__()
        
        self._payload_sub=rospy.Subscriber('payload', PayloadArray, self._payload_cb)
 
    def _payload_cb(self, msg):
                        
        for p in msg.payloads:
            t=TransformStamped()
            t.header.frame_id=p.header.frame_id
            t.header.stamp=p.header.stamp
            t.child_frame_id=p.name
            t.transform.rotation = p.pose.orientation
            t.transform.translation = _point_to_vector3(p.pose.position)
            self.ros_listener.setTransform(t)
            
            for g in p.gripper_targets:
                t=TransformStamped()
                t.header.frame_id=p.name
                t.header.stamp=p.header.stamp
                t.child_frame_id=g.name
                t.transform.rotation = g.pose.orientation
                t.transform.translation = _point_to_vector3(g.pose.position)
                self.ros_listener.setTransform(t)
                
            for m in p.markers:
                t=TransformStamped()
                t.header.frame_id=p.name
                t.header.stamp=p.header.stamp
                t.child_frame_id=m.name
                t.transform.rotation = m.pose.orientation
                t.transform.translation = _point_to_vector3(m.pose.position)
                self.ros_listener.setTransform(t)
            
        for p_t in msg.payload_targets:
            t=TransformStamped()
            t.header.frame_id=p_t.header.frame_id
            t.header.stamp=p_t.header.stamp
            t.child_frame_id=p_t.name
            t.transform.rotation = p_t.pose.orientation
            t.transform.translation = _point_to_vector3(p_t.pose.position)
            self.ros_listener.setTransform(t)            
        
        for p_m in msg.link_markers:
            for m in p_m.markers:
                t=TransformStamped()
                t.header.frame_id=p_m.header.frame_id
                t.header.stamp=p_m.header.stamp
                t.child_frame_id=m.name
                t.transform.rotation = m.pose.orientation
                t.transform.translation = _point_to_vector3(m.pose.position)
                self.ros_listener.setTransform(t)   
        
