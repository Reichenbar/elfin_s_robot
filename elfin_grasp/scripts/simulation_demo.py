#!/usr/bin/env python

from elfin_basic_api.srv import ReachEEPose, ReachEEPoseRequest
from elfin_basic_api.srv import ReachJointState, ReachJointStateRequest
from collision_free_motion_planning.srv import AddCollisionObj, AddCollisionObjRequest
from collision_free_motion_planning.srv import AddBox, AddBoxRequest
from collision_free_motion_planning.srv import RemoveCollisionObj, RemoveCollisionObjRequest
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import os
import math



def reach_ee_pose(ee_pose: list):
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


def add_box(box_pose: list):
    add_box_proxy = rospy.ServiceProxy('add_box', AddBox)
    rospy.wait_for_service('add_box')
    add_box_req = AddBoxRequest()
    # mesh pose setting
    collision_obj_pose_stamped = PoseStamped()
    collision_obj_pose_stamped.header.stamp=rospy.get_rostime()
    collision_obj_pose_stamped.header.frame_id = "world"
    collision_obj_pose_stamped.pose.position.x = box_pose[0]
    collision_obj_pose_stamped.pose.position.y = box_pose[1]
    collision_obj_pose_stamped.pose.position.z = box_pose[2]
    collision_obj_pose_stamped.pose.orientation.x = box_pose[3]
    collision_obj_pose_stamped.pose.orientation.y = box_pose[4]
    collision_obj_pose_stamped.pose.orientation.z = box_pose[5]
    collision_obj_pose_stamped.pose.orientation.w = box_pose[6]
    # service setting
    add_box_req.pose_stamped = collision_obj_pose_stamped
    add_box_res = add_box_proxy(add_box_req)
    if add_box_res.success:
        rospy.loginfo("Service [add_box] call was successful")
    else:
        rospy.logwarn("Service [add_box] call failed")



def remove_collision_obj(obj_name: str):
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
    input("Add the washer to the planning scene.")
    add_collision_obj("washer", [2.0, 0.0, 1.5, 0.0, 0.0, -0.707, 0.707], "../collision_objects/washer.stl")
    input("The manipulator goes to the initial state.")
    reach_joint_state([-1.57, -0.79, 0.79, -1.57, -1.57, 0.0])
    input("Add baskets to the planning scene.")
    add_collision_obj("filling_basket_1", [0.0, 1.35, 0.6, 0.707, 0.0, 0.0, 0.707], "../collision_objects/basket.stl")
    # add_collision_obj("filling_basket_2", [0.0, 0.735, 0.6, 0.707, 0.0, 0.0, 0.707], "../collision_objects/basket.stl")
    # add_collision_obj("filling_basket_3", [0.0, -0.735, 0.6, 0.707, 0.0, 0.0, 0.707], "../collision_objects/basket.stl")
    # add_collision_obj("filling_basket_4", [0.0, 1.965, 1.07, 0.707, 0.0, 0.0, 0.707], "../collision_objects/basket_three_faces.stl")

    input("The base moves away from the washer.")
    reach_base_state([-0.288, 0.0])
    input("The manipulator is ready to pick the bag.")
    reach_joint_state([-3.14, -1.66, 2.18, -2.09, -1.57, 0.0])
    input("The manipulator picks the bag from baskets.")
    reach_joint_state([item/180*math.pi for item in [-172, -59, 81, -118, -90, 0]])
    input("The manipulator is ready to pick the bag back from washer.")
    reach_joint_state([item/180*math.pi for item in [-180, -62, 81, -118, -90, 0]])
    input("The manipulator is ready to place the bag.")
    reach_joint_state([-3.14, -1.66, 2.18, -2.09, -1.57, 0.0])


    input("The manipulator is ready to place the bag.")
    reach_joint_state([-1.57, -1.66, 2.18, -2.09, -1.57, 0.0])
    input("The manipulator goes to the initial state.")
    reach_joint_state([-1.57, -0.79, 0.79, -1.57, -1.57, 0.0])
    input("The base moves away from the washer.")
    reach_base_state([0.0, 0.0])






















    input("The manipulator is ready to pick the bag from baskets.")
    reach_joint_state([item/180*math.pi for item in [-92, -48, 33, -45, -90, 3]])
    input("The manipulator picks the bag from baskets.")
    # reach_ee_pose([0.2587, 1.484, 0.7719, 0.7027, 0.0695, 0.6179, 0.3458])
    reach_joint_state([item/180*math.pi for item in [-92, -38, 68, -45, -90, 3]])
    input("The manipulator is ready to transfer the bag to the washing machine.")
    reach_joint_state([item/180*math.pi for item in [-92, -48, 33, -5, 29, 3]])
    # input("The manipulator goes back to the initial state.")
    # reach_joint_state([3.14, -1.57, 1.57, -1.57, -1.57, 0.0])

    input("The base moves away from the washer.")
    reach_base_state([-0.288, 0.0])
    input("The manipulator is ready to place the bag.")
    reach_joint_state([-3.14, -1.66, 2.18, -2.09, -1.57, 0.0])

    # # =====
    # input("The manipulator is ready to place the bag.")
    # reach_joint_state([item/180*math.pi for item in [-180, -80, 100, -110, -90, 0]])
    # input("The manipulator is ready to place the bag.")
    # reach_joint_state([item/180*math.pi for item in [-180, -60, 70, -100, -90, 0]])
    # # ======

    input("The manipulator is further ready to place the bag.")
    reach_joint_state([-3.14, -0.79, 0.79, -1.57, -1.57, 0.0])
    input("The base moves close to the washer.")
    # reach_base_state([0.288, 0.0])
    # reach_base_state([0.288, 0.0])
    reach_base_state([0.0, 0.0])

    input("The base moves away from the washer.")
    reach_base_state([-0.288, 0.0])
    input("The manipulator is ready to place the bag.")
    reach_joint_state([-3.14, -1.66, 2.18, -2.09, -1.57, 0.0])
    input("The manipulator goes to the initial state.")
    reach_joint_state([-1.57, -0.79, 0.79, -1.57, -1.57, 0.0])
    input("The base moves back to the initial position.")
    reach_base_state([0.0, 0.0])


    # input("The manipulator picks the bag.")
    # reach_ee_pose([2.416, 0.461, 1.823, 0.340, 0.099, -0.144, 0.924])
    # # reach_joint_state([item/180*math.pi for item in [0, -90, 90, -90, -70, -20]])
    # input("The manipulator goes back to the initial state.")
    # reach_joint_state([3.14, -1.57, 1.57, -1.57, -1.57, 0.0])


    # input("The base moves away from the washer.")
    # reach_base_state([0.0, 0.0])
    # input("Add baskets to the planning scene.")
    # add_collision_obj("retrieving_basket", [0.51, -1.36, 0.8, 0.0, 0.707, 0.707, 0.0], "../collision_objects/basket.stl")
    # input("The manipulator places the bag.")
    # reach_joint_state([item/180*math.pi for item in [90, -90, 100, -50, -90, 0]])
    # input("The manipulator goes to the initial state.")
    # reach_joint_state([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

    input("Remove the washer from the planning scene.")
    remove_collision_obj("washer")
    remove_collision_obj("filling_basket_1")


def check_box_pose():
    rospy.init_node("simulation_demo", anonymous=True)
    reach_joint_state([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    import numpy as np
    box_pos_example = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    corner_pos_example = np.array([[-0.2532, 1.1227, 1.1731, 0.5261, 0.5071, 0.6702, -0.1300],
                                 [0.2587, 1.1340, 1.1719, 0.7027, 0.0695, 0.6179, 0.3458],
                                 [0.2358, 0.8215, 1.1521, 0.3930, -0.5468, 0.0993, 0.7326],
                                 [-0.2396, 0.9006, 1.1598, -0.1575, -0.7121, -0.4386, 0.5251]])
    ee_pos_example = np.array([-0.2848, 1.0479, 1.628, 0.4402, 0.5534, 0.6579, -0.2591])
    corner_pos_local = corner_pos_example-box_pos_example
    ee_pos_local = ee_pos_example-box_pos_example

    box_pos = np.array([0.0, 1.35, 0.6, 0.0, 0.0, 0.0, 0.0]) # y-max 1.35, y-min 0.8
    corner_pos_global = corner_pos_local+box_pos
    print(corner_pos_global)
    ee_pos_global = ee_pos_local+box_pos
    input("1")
    add_collision_obj("filling_basket", box_pos[:3].tolist()+[0.707, 0.0, 0.0, 0.707], "../collision_objects/basket.stl")
    input("2")
    # reach_ee_pose(ee_pos_global.tolist())
    # reach_joint_state([item/180*math.pi for item in [-92, -48, 33, -5, 29, 3]]) # y-max
    reach_joint_state([item/180*math.pi for item in [-92, -83, 80, 3, 27, 0]]) # y-min
    for i in range(corner_pos_global.shape[0]):
        input(f"Corner {i}")
        reach_ee_pose(corner_pos_global[i, :].tolist())
    input("3")
    remove_collision_obj()






if __name__ == "__main__":
    main()