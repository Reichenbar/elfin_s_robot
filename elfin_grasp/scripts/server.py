#!/usr/bin/env python

from elfin_grasp.srv import ReachEEPose, ReachEEPoseResponse
from elfin_grasp.srv import ReachJointState, ReachJointStateResponse
from elfin_grasp.srv import AddCollisionObj, AddCollisionObjResponse
from elfin_grasp.srv import AddBox, AddBoxResponse
from elfin_grasp.srv import RemoveCollisionObj, RemoveCollisionObjResponse
import rospy
import moveit_commander
import sys
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, SolidPrimitive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

# from validation import test
# from srv.testing import test


class server:   
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)    
        self.robot = moveit_commander.RobotCommander()
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        arm_group_name = "elfin_arm"
        self.arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)
        base_group_name = "elfin_base"
        self.base_move_group = moveit_commander.MoveGroupCommander(base_group_name)

        rospy.loginfo(f"Available planning groups are {self.robot.get_group_names()}")
        rospy.loginfo(f"The planning frame for {arm_group_name} is {self.arm_move_group.get_planning_frame()}")
        rospy.loginfo(f"The end-effector link for {arm_group_name} is {self.arm_move_group.get_end_effector_link()}")
        rospy.loginfo(f"The planning frame for {base_group_name} is {self.base_move_group.get_planning_frame()}")
        rospy.loginfo(f"The end-effector link for {base_group_name} is {self.base_move_group.get_end_effector_link()}")

        self.joints_pub=rospy.Publisher('elfin_basic_api/joint_goal', JointState, queue_size=1)
        self.cart_pub=rospy.Publisher('elfin_basic_api/cart_goal', PoseStamped, queue_size=1)

        self.pub_co = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)

        reach_ee_pose_srv = rospy.Service('reach_ee_pose', ReachEEPose, self.reach_ee_pose_cb)
        reach_joint_state_srv = rospy.Service('reach_joint_state', ReachJointState, self.reach_joint_state_cb)
        add_collision_obj_srv = rospy.Service('add_collision_obj', AddCollisionObj, self.add_collision_cb)
        add_box_srv = rospy.Service('add_box', AddBox, self.add_box_cb)
        remove_collision_obj_srv = rospy.Service('remove_collision_obj', RemoveCollisionObj, self.remove_collision_cb)
        reach_base_state_srv = rospy.Service('reach_base_state', ReachJointState, self.reach_base_state_cb)


        rospy.loginfo(f"The service {reach_ee_pose_srv.resolved_name} is now available.") # Waiting for service /gazebo/spawn_urdf_mode
        rospy.loginfo(f"The service {reach_joint_state_srv.resolved_name} is now available.")
        rospy.loginfo(f"The service {add_collision_obj_srv.resolved_name} is now available.")
        rospy.loginfo(f"The service {remove_collision_obj_srv.resolved_name} is now available.")
        rospy.loginfo(f"The service {reach_base_state_srv.resolved_name} is now available.")


    def reach_ee_pose_cb(self, req):
        ## ----------
        # self.arm_move_group.set_pose_target(req.pose)
        # success = self.arm_move_group.go(wait=True)
        # self.arm_move_group.stop()
        # self.arm_move_group.clear_pose_targets()
        ## -----------
        self.cart_pub.publish(req.pose_stamped)
        return ReachEEPoseResponse(True)
    
    def reach_joint_state_cb(self, req):
        self.joints_pub.publish(req.joint_state)
        return ReachJointStateResponse(True)

    def add_collision_cb(self, req):
        self.planning_scene.add_mesh(req.name, req.pose_stamped, req.filename)
        rospy.loginfo(f"Add {req.name} into the planning scene.")
        return AddCollisionObjResponse(True)
    
    def add_box_cb(self, req):
        # Define the dimensions of the outer box
        outer_box_size = [0.855, 0.615, 0.470]  # [length, width, height]

        # Define the dimensions of the inner box (cavity)
        inner_box_size = [0.765, 0.555, 0.470]  # [length, width, height]

        # Define the pose of the box (position and orientation)
        box_pose = req.pose_stamped
        # box_pose.header.frame_id = self.arm_move_group.get_planning_frame()
        # box_pose.pose.position.x = 1.5  # x-coordinate
        # box_pose.pose.position.y = 0.0  # y-coordinate
        # box_pose.pose.position.z = 0.0  # z-coordinate
        # box_pose.pose.orientation.x = 0.0  # Quaternion orientation
        # box_pose.pose.orientation.y = 0.0  # Quaternion orientation
        # box_pose.pose.orientation.z = 0.0  # Quaternion orientation
        # box_pose.pose.orientation.w = 1.0  # Quaternion orientation

        # Create the outer box collision object
        outer_box = CollisionObject()
        outer_box.id = "outer_box"
        outer_box.operation = outer_box.ADD
        outer_box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=outer_box_size)]
        outer_box.primitive_poses = [box_pose.pose]



        # Create the inner box collision object (cavity)
        inner_box = CollisionObject()
        inner_box.id = "inner_box"
        inner_box.operation = inner_box.REMOVE  # Remove the inner box from the outer box
        inner_box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=inner_box_size)]
        inner_box.primitive_poses = [box_pose.pose]

        # Add the outer box to the planning scene
        # self.planning_scene.add_object(outer_box)
        rospy.sleep(2)
        self.pub_co.publish(outer_box)

        # Add the inner box (cavity) to the planning scene
        # self.planning_scene.add_object(inner_box)
        self.pub_co.publish(inner_box)
    
    def remove_collision_cb(self, req):
        self.planning_scene.remove_world_object(req.name)
        rospy.loginfo(f"Remove {req.name} from the planning scene.")
        return RemoveCollisionObjResponse(True)
    
    def reach_base_state_cb(self, req):
        self.base_move_group.go(req.joint_state, wait=True)
        self.base_move_group.stop()
        return ReachJointStateResponse(True)


def main():
    rospy.init_node("server", anonymous=True)
    server()
    rospy.spin()


if __name__ == "__main__":
    main()