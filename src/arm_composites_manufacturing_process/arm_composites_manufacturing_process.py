# Copyright (c) 2018, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Rensselaer Polytechnic Institute, nor Wason 
#       Technology LLC, nor the names of its contributors may be used to 
#       endorse or promote products derived from this software without 
#       specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import copy
import rospy
import actionlib

import general_robotics_toolbox as rox
import general_robotics_toolbox.urdf as urdf
import general_robotics_toolbox.ros_msg as rox_msg
from general_robotics_toolbox import ros_tf as tf

import rpi_abb_irc5.ros.rapid_commander as rapid_node_pkg
import safe_kinematic_controller.ros.commander as controller_commander_pkg
from rpi_arm_composites_manufacturing_process.msg import ProcessState
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal
from rpi_arm_composites_manufacturing_process.msg import PayloadArray
from industrial_payload_manager.payload_transform_listener import PayloadTransformListener
from industrial_payload_manager.srv import UpdatePayloadPose, UpdatePayloadPoseRequest, \
    GetPayloadArray, GetPayloadArrayRequest
import time
import sys
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, MoveItErrorCodes
import os
from rpi_arm_composites_manufacturing_process.msg import Payload, PayloadTarget, PayloadArray, ArucoGridboard, ProcessState
import threading
from moveit_commander import PlanningSceneInterface
import traceback
import resource_retriever
import urlparse
from urdf_parser_py.urdf import URDF
from tf.msg import tfMessage
from visualization_msgs.msg import Marker, MarkerArray


class ProcessControllerPayload(object):
    def __init__(self, payload_msg, ros_id):
        self.payload_msg=payload_msg
        self.ros_id=ros_id
        self.attached_link=None

class ProcessController(object):
    def __init__(self, disable_ft=False):
        self.urdf=URDF.from_parameter_server()
        self.overhead_vision_client=actionlib.SimpleActionClient("recognize_objects", ObjectRecognitionAction)
        self.execute_trajectory_action=actionlib.SimpleActionClient("execute_trajectory",ExecuteTrajectoryAction)
        self.rapid_node = rapid_node_pkg.RAPIDCommander()
        self.controller_commander=controller_commander_pkg.ControllerCommander()
        self.state='init'
        self.current_target=None
        self.current_payload=None
        self.available_payloads={'leeward_mid_panel': 'leeward_mid_panel','leeward_tip_panel':'leeward_tip_panel'}
        self.desired_controller_mode=self.controller_commander.MODE_AUTO_TRAJECTORY
        self.speed_scalar=1.0
        self.disable_ft=disable_ft
       
        self.tf_listener=PayloadTransformListener()
        self._process_state_pub = rospy.Publisher("process_state", ProcessState, queue_size=100, latch=True)
        self.publish_process_state()

        self.update_payload_pose_srv=rospy.ServiceProxy("update_payload_pose", UpdatePayloadPose)
        self.get_payload_array_srv=rospy.ServiceProxy("get_payload_array", GetPayloadArray)
        
        self.plan_dictionary={}
    
    def _vision_get_object_pose(self, key):
        self.overhead_vision_client.wait_for_server()
        
        goal=ObjectRecognitionGoal(False, [-1e10,1e10,-1e10,1e10])
        self.overhead_vision_client.send_goal(goal)
        self.overhead_vision_client.wait_for_result()
        ret=self.overhead_vision_client.get_result()
                        
        for r in ret.recognized_objects.objects:
            if r.type.key == key:
                rox_pose=rox_msg.msg2transform(r.pose.pose.pose)
                rox_pose.parent_frame_id=r.pose.header.frame_id
                rox_pose.child_frame_id=key
                return rox_pose
            
        raise Exception("Requested object not found")
    
    def _vision_get_object_gripper_target_pose(self, key):
        
        object_pose=self._vision_get_object_pose(key)
                
        tag_rel_pose = self.tf_listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))        
        return object_pose * tag_rel_pose, object_pose
    
    def _tf_get_object_gripper_target_pose(self, key):
        
        payload=self._get_payload(key)
        if payload.confidence < 0.8:
            raise Exception("Payload confidence too low for tf lookup")
        
        object_pose = self.tf_listener.lookupTransform("/world", key, rospy.Time(0))
                
        tag_rel_pose = self.tf_listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))        
        return object_pose * tag_rel_pose, object_pose
    
    
    
    def get_payload_pickup_ft_threshold(self, payload):
        if self.disable_ft:
            return []
        return self._get_payload(payload).gripper_targets[0].pickup_ft_threshold
    
    def _active_client(self):
        self.state="moving"
        #self.publish_process_state()

    def _finished_client(self,state,result):
        #if(state== actionlib.GoalStatus.SUCCEEDED):

        self.publish_process_state()
        if(result!=MoveItErrorCodes.SUCCESS):
            rospy.loginfo(MoveItErrorCodes.SUCCESS)
            rospy.loginfo("MoveItErrorCode generated: %s",str(result))

    def get_state(self):
        return self.state
    
    def get_current_pose(self):
        return self.controller_commander.get_current_pose()

    def plan_to_reset_position(self):
        #TODO: Implement reset movement in process controller
        rospy.loginfo("Planning to reset position")
        self.state="reset_position"
        self.plan_dictionary['reset']=path
        self.publish_process_state()

    def plan_pickup_prepare(self, target_payload):
        
        #TODO: check state and payload
        
        rospy.loginfo("Begin pickup_prepare for payload %s", target_payload)
        
        object_target, object_pose=self._vision_get_object_gripper_target_pose(target_payload)
        
        self._update_payload_pose(target_payload, object_pose, confidence=0.8)
        
        rospy.loginfo("Found payload %s at pose %s", target_payload, object_target)
        
        self.pose_target=copy.deepcopy(object_target)
        pose_target=self.pose_target
        pose_target.p[2] += 0.5
        
        rospy.loginfo("Prepare16 pickup %s at pose %s", target_payload, object_target)
        

        path=self.controller_commander.plan(pose_target)

        self.current_target=target_payload
        self.state="plan_pickup_prepare"
        self.plan_dictionary['pickup_prepare']=path
        
        rospy.loginfo("Finish pickup prepare for payload %s", target_payload)
        self.publish_process_state()

    def move_pickup_prepare(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, self.speed_scalar,[], [])
        result=None
        self.state="pickup_prepare"
        goal=ExecuteTrajectoryGoal()
        goal.trajectory=self.plan_dictionary['pickup_prepare']
        self.execute_trajectory_action.send_goal(goal,active_cb=self._active_client,done_cb=self._finished_client)
        #self.controller_commander.async_execute(self.plan_dictionary['pickup_prepare'],result)


    def plan_pickup_lower(self):

        #TODO: check change state and target
        
        rospy.loginfo("Begin pickup_lower for payload %s", self.current_target)
        

        object_target, _=self._tf_get_object_gripper_target_pose(self.current_target)
        pose_target2=copy.deepcopy(object_target)
        pose_target2.p[2] += 0.15    


        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)

        self.state="plan_pickup_lower"
        self.plan_dictionary['pickup_lower']=path
        rospy.loginfo("Finish pickup_lower for payload %s", self.current_target)
        self.publish_process_state()

    def move_pickup_lower(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.8*self.speed_scalar,[], self.get_payload_pickup_ft_threshold(self.current_target))
        result=None
        self.state="pickup_lower"
        goal=ExecuteTrajectoryGoal()
        goal.trajectory=self.plan_dictionary['pickup_lower']
        self.execute_trajectory_action.send_goal(goal,active_cb=self._active_client,done_cb=self._finished_client)
        #self.controller_commander.execute(self.plan_dictionary['pickup_lower'])


    def plan_pickup_grab_first_step(self):
        #TODO: check change state and target
        
        rospy.loginfo("Begin pickup_grab for payload %s", self.current_target)
               
        self.object_target, _=self._tf_get_object_gripper_target_pose(self.current_target)
        pose_target2=copy.deepcopy(self.object_target)
        pose_target2.p[2] -= 0.15   

        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)
        self.state="plan_pickup_grab_first_step"
        self.plan_dictionary['pickup_grab_first_step']=path
        self.publish_process_state()

    def move_pickup_grab_first_step(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.4*self.speed_scalar, [],\
                                                      self.get_payload_pickup_ft_threshold(self.current_target))
        result=None

        self.state="pickup_grab_first_step"
        try:
            goal=ExecuteTrajectoryGoal()
            goal.trajectory=self.plan_dictionary['pickup_grab_first_step']
            self.execute_trajectory_action.send_goal(goal,active_cb=self._active_client,done_cb=self._finished_client)
            #self.controller_commander.execute(self.plan_dictionary['pickup_grab_first_step'])
        except Exception as err:
            print err


    def plan_pickup_grab_second_step(self):
        self.rapid_node.set_digital_io("Vacuum_enable", 1)
        time.sleep(1)        
        
        #TODO: check vacuum feedback to make sure we have the panel
        
        world_to_panel_tf=self.tf_listener.lookupTransform("world", self.current_target, rospy.Time(0))
        world_to_gripper_tf=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
        panel_to_gripper_tf=world_to_gripper_tf.inv()*world_to_panel_tf

        self.current_payload=self.current_target
        self.current_target=None
        
        self._update_payload_pose(self.current_payload, panel_to_gripper_tf, "vacuum_gripper_tool", 0.5)
        self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT,1,[],[])
        time.sleep(1)

        pose_target2=copy.deepcopy(self.object_target)
        pose_target2.p[2] += 0.15   
        

          
        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)

        self.state="plan_pickup_grab_second_step"
        self.plan_dictionary['pickup_grab_second_step']=path
        rospy.loginfo("Finish pickup_grab for payload %s", self.current_target)
        self.publish_process_state()

    def move_pickup_grab_second_step(self):

        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.4*self.speed_scalar,[], [])
        result=None
        self.state="pickup_grab_second_step"
        goal=ExecuteTrajectoryGoal()
        goal.trajectory=self.plan_dictionary['pickup_grab_second_step']
        self.execute_trajectory_action.send_goal(goal,active_cb=self._active_client,done_cb=self._finished_client)
        #self.controller_commander.execute(self.plan_dictionary['pickup_grab_second_step'])


    def plan_pickup_raise(self):
        
        #TODO: check change state and target
        
        rospy.loginfo("Begin pickup_raise for payload %s", self.current_payload)
        
        #Just use gripper position for now, think up a better way in future
        object_target=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
        pose_target2=copy.deepcopy(object_target)
        pose_target2.p[2] += 0.8
        #pose_target2.p = np.array([-0.02285,-1.840,1.0])
        #pose_target2.R = rox.q2R([0.0, 0.707, 0.707, 0.0])


          
        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)
        
        self.state="plan_pickup_raise"
        self.plan_dictionary['pickup_raise']=path
        rospy.loginfo("Finish pickup_raise for payload %s", self.current_target)
        self.publish_process_state()

    def move_pickup_raise(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.8*self.speed_scalar, [], [])
        result=None
        self.state="pickup_raise"
        goal=ExecuteTrajectoryGoal()
        goal.trajectory=self.plan_dictionary['pickup_raise']
        self.execute_trajectory_action.send_goal(goal,active_cb=self._active_client,done_cb=self._finished_client)
        #self.controller_commander.async_execute(self.plan_dictionary['pickup_raise'],result)


    def plan_transport_payload(self, target):
        
        #TODO: check state and payload
        
        rospy.loginfo("Begin transport_panel for payload %s to %s", self.current_payload, target)
        
        panel_target_pose = self.tf_listener.lookupTransform("world", target, rospy.Time(0))        
        panel_gripper_pose = self.tf_listener.lookupTransform(self.current_payload, "vacuum_gripper_tool", rospy.Time(0))        
        pose_target=panel_target_pose * panel_gripper_pose
        pose_target.p = [2.197026484647054, 1.2179574262842452, 0.12376598588449844]
        pose_target.R = np.array([[-0.99804142,  0.00642963,  0.06222524], [ 0.00583933,  0.99993626, -0.00966372], [-0.06228341, -0.00928144, -0.99801535]])
        pose_target.p[2] += 0.35


        plan=self.controller_commander.plan(pose_target)
        
        self.current_target=target
        self.state="plan_transport_panel"
        self.plan_dictionary['transport_payload']=plan
        rospy.loginfo("Finish transport_panel for payload %s to %s", self.current_payload, target)
        self.publish_process_state()

    def move_transport_payload(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.8*self.speed_scalar, [], [])
        result=None
        self.state="transport_panel"
        self.controller_commander.async_execute(self.plan_dictionary['transport_payload'],result)
        self.publish_process_state()

    def plan_place_lower(self):
        
        #TODO: check state and payload
        
        rospy.loginfo("Begin place_lower for payload %s to %s", self.current_payload, self.current_target)
        
        panel_target_pose = self.tf_listener.lookupTransform("world", self.current_target, rospy.Time(0))        
        panel_gripper_pose = self.tf_listener.lookupTransform(self.current_payload, "vacuum_gripper_tool", rospy.Time(0))        
        pose_target=panel_target_pose * panel_gripper_pose
                
        pose_target.p[2] += 0.15  

        path=self.controller_commander.plan(pose_target)
                
        self.state="plan_place_lower"
        self.plan_dictionary['place_lower']=path
        rospy.loginfo("Finish place_lower for payload %s to %s", self.current_payload, self.current_target)
        self.publish_process_state()

    def move_place_lower(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.8*self.speed_scalar, [], [])
        result=None
        self.state="place_lower"
        self.controller_commander.async_execute(self.plan_dictionary['place_lower'],result)
        self.publish_process_state()

    def plan_place_set_first_step(self):
        
        #TODO: check change state and target

        rospy.loginfo("Begin place_set for payload %s target %s", self.current_payload, self.current_target)

        panel_target_pose = self.tf_listener.lookupTransform("world", self.current_target, rospy.Time(0))
        panel_gripper_pose = self.tf_listener.lookupTransform(self.current_payload, "vacuum_gripper_tool", rospy.Time(0))
        pose_target=panel_target_pose * panel_gripper_pose
        pose_target2=copy.deepcopy(pose_target)
        pose_target2.p[2] -= 0.15



        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)
        self.state="plan_place_set_first_step"
        self.plan_dictionary['place_set_first_step']=path
        self.publish_process_state()

    def move_place_set_first_step(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.4*self.speed_scalar, [], \
                                                      self.get_payload_pickup_ft_threshold(self.current_target))
        result=None
        self.state="place_set_first_step"
        self.controller_commander.async_execute(self.plan_dictionary['place_set_first_step'],result)
        self.rapid_node.set_digital_io("Vacuum_enable", 0)
        time.sleep(1)
        self.publish_process_state()

    def plan_place_set_second_step(self):

        #TODO: check vacuum feedback to make sure we have the panel

        gripper_to_panel_tf=self.tf_listener.lookupTransform("vacuum_gripper_tool", self.current_payload, rospy.Time(0))
        world_to_gripper_tf=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
        world_to_panel_nest_tf=self.tf_listener.lookupTransform("world", "panel_nest", rospy.Time(0))
        panel_to_nest_tf=world_to_panel_nest_tf.inv()*world_to_gripper_tf*gripper_to_panel_tf

        self._update_payload_pose(self.current_payload, panel_to_nest_tf, "panel_nest", 0.5)

        self.current_payload=None
        self.current_target=None

        time.sleep(1)

        pose_target2=copy.deepcopy(pose_target)
        pose_target2.p[2] += 0.15


        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)

        self.state="plan_place_set_second_step"
        self.plan_dictionary['place_set_second_step']=path
        rospy.loginfo("Finish place_set for payload %s", self.current_target)
        self.publish_process_state()

    def move_place_set_second_step(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.4*self.speed_scalar, [], [])
        result=None
        self.state="place_set_second_step"
        self.controller_commander.async_execute(self.plan_dictionary['place_set_second_step'],result)
        self.publish_process_state()


        
    def plan_place_raise(self):
        
        #TODO: check change state and target
        
        rospy.loginfo("Begin place_raise for payload %s", self.current_payload)
        
        #Just use gripper position for now, think up a better way in future
        object_target=self.tf_listener.lookupTransform("world", "vacuum_gripper_tool", rospy.Time(0))
        pose_target2=copy.deepcopy(object_target)
        pose_target2.p[2] += 0.5
        

        path=self.controller_commander.compute_cartesian_path(pose_target2, avoid_collisions=False)


       
        self.state="plan_place_raise"
        self.plan_dictionary['place_raise']=path
        rospy.loginfo("Finish place_raise for payload %s", self.current_target)
        self.publish_process_state()

    def move_place_raise(self):
        self.controller_commander.set_controller_mode(self.desired_controller_mode, 0.8*self.speed_scalar, [], [])
        result=None
        self.state="place_raise"
        self.controller_commander.async_execute(self.plan_dictionary['place_raise'],result)
        self.publish_process_state()

    def _fill_process_state(self):
        s=ProcessState()
        s.state=self.state if self.state is not None else ""
        s.payload=self.current_payload if self.current_payload is not None else ""
        s.target=self.current_target if self.current_target is not None else ""
        return s

    def publish_process_state(self):
        s=self._fill_process_state()
        self._process_state_pub.publish(s)

    def place_lower_temp(self):
        UV = np.zeros([32,2])
        P = np.zeros([32,3])
    
    def _update_payload_pose(self, payload_name, pose, parent_frame_id = None, confidence = 0.1):
        
        payload = self._get_payload(payload_name)
        
        if parent_frame_id is None:
                parent_frame_id = payload.header.frame_id
                
        parent_tf = self.tf_listener.lookupTransform(parent_frame_id, pose.parent_frame_id, rospy.Time(0))
        pose2=parent_tf.inv() * pose
        
        req=UpdatePayloadPoseRequest()
        req.name=payload_name
        req.pose=rox_msg.transform2pose_msg(pose2)
        req.header.frame_id=parent_frame_id            
        req.confidence = confidence
        
        res=self.update_payload_pose_srv(req)
        if not res.success:
            raise Exception("Could not update payload pose")
        
    def _get_payload(self, payload_name):
        payload_array_res = self.get_payload_array_srv(GetPayloadArrayRequest([payload_name]))
        if len(payload_array_res.payload_array.payloads) != 1 or payload_array_res.payload_array.payloads[0].name != payload_name:
            raise Exception("Invalid payload specified")
        
        return payload_array_res.payload_array.payloads[0]

