#!/usr/bin/env python

import os
import math
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from elfin_basic_api.srv import ReachEEPose, ReachEEPoseRequest
from elfin_basic_api.srv import ReachJointState, ReachJointStateRequest
from elfin_grasp.srv import AddCollisionObj, AddCollisionObjRequest
from elfin_grasp.srv import RemoveCollisionObj, RemoveCollisionObjRequest

def reach_ee_pose(ee_pose: list):
    """
    Send the desired end-effector pose.

    Args:
        ee_pose: length 7, [position (x, y, z), orientation (x, y, z, w)].
    """
    reach_ee_pose_proxy = rospy.ServiceProxy('/elfin_basic_api/reach_ee_pose', ReachEEPose)
    rospy.wait_for_service('/elfin_basic_api/reach_ee_pose')
    reach_ee_pose_req = ReachEEPoseRequest()

    ee_pose_stamped = PoseStamped()
    ee_pose_stamped.header.stamp=rospy.get_rostime()
    ee_pose_stamped.header.frame_id = "world"
    ee_pose_stamped.pose.position.x = ee_pose[0]
    ee_pose_stamped.pose.position.y = ee_pose[1]
    ee_pose_stamped.pose.position.z = ee_pose[2]
    ee_pose_stamped.pose.orientation.x = ee_pose[3]
    ee_pose_stamped.pose.orientation.y = ee_pose[4]
    ee_pose_stamped.pose.orientation.z = ee_pose[5]
    ee_pose_stamped.pose.orientation.w = ee_pose[6]

    reach_ee_pose_req.pose_stamped = ee_pose_stamped
    reach_ee_pose_res = reach_ee_pose_proxy(reach_ee_pose_req)
    if reach_ee_pose_res.success:
        rospy.loginfo("Service [reach_ee_pose] call was successful")
    else:
        rospy.logwarn("Service [reach_ee_pose] call failed")

def reach_joint_state(joint_pos: list):
    """
    Send the desired robot joint state.

    Args:
        joint_pos: length 6.
    """
    reach_joint_state_proxy = rospy.ServiceProxy('/elfin_basic_api/reach_joint_state', ReachJointState)
    rospy.wait_for_service('/elfin_basic_api/reach_joint_state')
    reach_joint_state_req = ReachJointStateRequest()

    joint_state=JointState()
    joint_state.name=['elfin_joint1', 'elfin_joint2', 'elfin_joint3', 
                      'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
    joint_state.position = joint_pos
    joint_state.header.stamp=rospy.get_rostime()

    reach_joint_state_req.joint_state = joint_state
    reach_joint_state_res = reach_joint_state_proxy(reach_joint_state_req)
    if reach_joint_state_res.success:
        rospy.loginfo("Service [reach_joint_state] call was successful")
    else:
        rospy.logwarn("Service [reach_joint_state] call failed")


def add_collision_obj(mesh_name: str, mesh_pose: list, relative_mesh_path):
    """
    Add the collision object to the planning scene.

    Args:
        mesh_name: unique name in the planning scene.
        mesh_pose: length 7, [position (x, y, z), orientation (x, y, z, w)].
        relative_mesh_path: "../collision_objects/*.stl".
    """
    add_collision_obj_proxy = rospy.ServiceProxy('add_collision_obj', AddCollisionObj)
    rospy.wait_for_service('add_collision_obj')
    add_collision_obj_req = AddCollisionObjRequest()
    add_collision_obj_req.name = mesh_name
    # mesh file directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_file_path = os.path.join(script_dir, relative_mesh_path)
    add_collision_obj_req.filename = mesh_file_path
    # mesh pose setting
    collision_obj_pose_stamped = PoseStamped()
    collision_obj_pose_stamped.header.stamp=rospy.get_rostime()
    collision_obj_pose_stamped.header.frame_id = "world"
    collision_obj_pose_stamped.pose.position.x = mesh_pose[0]
    collision_obj_pose_stamped.pose.position.y = mesh_pose[1]
    collision_obj_pose_stamped.pose.position.z = mesh_pose[2]
    collision_obj_pose_stamped.pose.orientation.x = mesh_pose[3]
    collision_obj_pose_stamped.pose.orientation.y = mesh_pose[4]
    collision_obj_pose_stamped.pose.orientation.z = mesh_pose[5]
    collision_obj_pose_stamped.pose.orientation.w = mesh_pose[6]
    # service setting
    add_collision_obj_req.pose_stamped = collision_obj_pose_stamped
    add_collision_obj_res = add_collision_obj_proxy(add_collision_obj_req)
    if add_collision_obj_res.success:
        rospy.loginfo("Service [add_collision_obj] call was successful")
    else:
        rospy.logwarn("Service [add_collision_obj] call failed")


def remove_collision_obj(obj_name: str):
    """
    Remove the collision object from the planning scene.

    Args:
        obj_name: unique name in the planning scene.
    """
    remove_collision_obj_proxy = rospy.ServiceProxy('remove_collision_obj', RemoveCollisionObj)
    rospy.wait_for_service('remove_collision_obj')
    remove_collision_obj_req = RemoveCollisionObjRequest()
    remove_collision_obj_req.name = obj_name
    remove_collision_obj_res = remove_collision_obj_proxy(remove_collision_obj_req)
    if remove_collision_obj_res.success:
        rospy.loginfo("Service [remove_collision_obj] call was successful")
    else:
        rospy.logwarn("Service [remove_collision_obj] call failed")

def reach_base_state(joint_pos: list):
    """
    Send the desired base joint state.

    Args:
        joint_pos: length 2.
    """
    reach_joint_state_proxy = rospy.ServiceProxy('reach_base_state', ReachJointState)
    rospy.wait_for_service('reach_base_state')
    reach_joint_state_req = ReachJointStateRequest()

    joint_state=JointState()
    joint_state.name=['mobile_base_joint1', 'mobile_base_joint2']
    joint_state.position = joint_pos
    joint_state.header.stamp=rospy.get_rostime()

    reach_joint_state_req.joint_state = joint_state
    reach_joint_state_res = reach_joint_state_proxy(reach_joint_state_req)
    if reach_joint_state_res.success:
        rospy.loginfo("Service [reach_base_state] call was successful")
    else:
        rospy.logwarn("Service [reach_base_state] call failed")



def main():
    rospy.init_node("simulation_demo", anonymous=True)
    input("Home configuration.")
    reach_joint_state([item/180*math.pi for item in [90, -135, 120, -75, -90, 0]])
    input("Initial configuration.")
    reach_joint_state([item/180*math.pi for item in [90, -105, 85, -40, -90, 0]])
    input("Grasp configuration.")
    reach_joint_state([item/180*math.pi for item in [80, -65, 100, -75, -90, 0]])
    input("Back to initial configuration.")
    reach_joint_state([item/180*math.pi for item in [90, -105, 85, -40, -90, 0]])


    # # ================= Move the base =================
    # input("The base moves away from the washer.")
    # reach_base_state([-0.288, 0.0])
    # input("The base moves away from the washer.")
    # reach_base_state([0.0, 0.0])

    # # ================== Add collision objects ================
    # input("Add the washer and basket to the planning scene.")
    # add_collision_obj("washer", [2.0, 0.0, 1.5, 0.0, 0.0, -0.707, 0.707], "../collision_objects/washer.stl")
    # add_collision_obj("filling_basket_1", [0.0, 1.35, 0.6, 0.707, 0.0, 0.0, 0.707], "../collision_objects/basket.stl")
    # input("Remove the washer and basket from the planning scene.")
    # remove_collision_obj("washer")
    # remove_collision_obj("filling_basket_1")


if __name__ == "__main__":
    main()