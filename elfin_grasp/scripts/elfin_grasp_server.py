#!/usr/bin/env python

import rospy
import moveit_commander
import sys
from elfin_grasp.srv import ReachJointState, ReachJointStateResponse
from elfin_grasp.srv import AddCollisionObj, AddCollisionObjResponse
from elfin_grasp.srv import RemoveCollisionObj, RemoveCollisionObjResponse



class elfin_grasp_server:   
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)    
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        base_group_name = "elfin_base"
        self.base_move_group = moveit_commander.MoveGroupCommander(base_group_name)

        add_collision_obj_srv = rospy.Service('add_collision_obj', AddCollisionObj, self.add_collision_cb)
        remove_collision_obj_srv = rospy.Service('remove_collision_obj', RemoveCollisionObj, self.remove_collision_cb)
        reach_base_state_srv = rospy.Service('reach_base_state', ReachJointState, self.reach_base_state_cb)

        rospy.loginfo(f"The service {add_collision_obj_srv.resolved_name} is now available.")
        rospy.loginfo(f"The service {remove_collision_obj_srv.resolved_name} is now available.")
        rospy.loginfo(f"The service {reach_base_state_srv.resolved_name} is now available.")

    def add_collision_cb(self, req):
        self.planning_scene.add_mesh(req.name, req.pose_stamped, req.filename)
        rospy.loginfo(f"Add {req.name} into the planning scene.")
        return AddCollisionObjResponse(True)
    
    def remove_collision_cb(self, req):
        self.planning_scene.remove_world_object(req.name)
        rospy.loginfo(f"Remove {req.name} from the planning scene.")
        return RemoveCollisionObjResponse(True)
    
    def reach_base_state_cb(self, req):
        self.base_move_group.go(req.joint_state, wait=True)
        self.base_move_group.stop()
        return ReachJointStateResponse(True)


def main():
    rospy.init_node("elfin_grasp_server", anonymous=True)
    elfin_grasp_server()
    rospy.spin()


if __name__ == "__main__":
    main()