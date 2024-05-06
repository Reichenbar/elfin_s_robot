#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// to add customized collision objects
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <urdf_parser/urdf_parser.h>

int main(int argc, char** argv)
{
    // Setup
    // ^^^^^
    ros::init(argc, argv, "collision_free_motion_planning");
    ros::NodeHandle node_handle;
    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangeably.
    static const std::string PLANNING_GROUP = "elfin_arm";
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();
    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "Collision free motion planning", rvt::WHITE, rvt::XLARGE);
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Basic information
    // ^^^^^^^^^^^^^^^^^
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");


    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = -0.651933;
    target_pose1.position.y = -0.259244;
    target_pose1.position.z = 0.14457;
    target_pose1.orientation.x = 0.56966;
    target_pose1.orientation.y = -0.418962;
    target_pose1.orientation.z = 0.377576;
    target_pose1.orientation.w = 0.597824;
    move_group_interface.setPoseTarget(target_pose1);
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group_interface
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    // joint_group_positions[1] = -90/180.0*M_PI;  // -1/6 turn in radians
    joint_group_positions[3] = -90/180.0*M_PI;  // -1/6 turn in radians
    // move_group_interface.setJointValueTarget(joint_group_positions);
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    // if (success) {
    // move_group_interface.execute(my_plan);
    // } else {
    // ROS_ERROR_STREAM_NAMED("tutorial", "Unable to plan");
    // }
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Add collision objects
    // ^^^^^^^^^^^^^^^^^^^^^

    moveit_msgs::CollisionObject washer;
    washer.header.frame_id = move_group_interface.getPlanningFrame();
    // The id of the object is used to identify it
    washer.id = "washer";

    /* failed to load urdf file
    std::string urdf_file = "/home/clover/elfin/src/elfin_s_robot/collision_free_motion_planning/collision_objects/washer.urdf";
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_file);
    std::string link_name = "washer";
    urdf::LinkConstSharedPtr link = model->getLink(link_name);

    const urdf::Visual* visual = link->visual.get();
    urdf::Geometry* geometry = visual->geometry.get();
    const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geometry);
    shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename);
    */

    shapes::Mesh* m = shapes::createMeshFromResource("package://collision_free_motion_planning/collision_objects/complete_washer.stl");
    // washer.mesh_poses.push_back(washer_pose);
    washer.operation = washer.ADD;
    shape_msgs::Mesh washer_mesh;
    shapes::ShapeMsg washer_mesh_msg;
    shapes::constructMsgFromShape(m, washer_mesh_msg);
    washer_mesh = boost::get<shape_msgs::Mesh>(washer_mesh_msg);

    // A pose (specified relative to frame_id)
    geometry_msgs::Pose washer_pose;
    washer_pose.orientation.w = 0.707;
    washer_pose.orientation.x = 0.0;
    washer_pose.orientation.y = 0.0;
    washer_pose.orientation.z = 0.707;
    washer_pose.position.x =  -3; // -1.7
    washer_pose.position.y =  0.0;
    washer_pose.position.z =  -0.2;

    washer.meshes.push_back(washer_mesh);
    planning_scene_interface.applyCollisionObject(washer);

    ROS_INFO("Add a washer into the world");

    // moveit_msgs::PlanningScene planning_scene;
    // planning_scene.world.collision_objects.push_back(collision_object);
    // planning_scene.is_diff = true;
    // planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    // planning_scene_diff_publisher.publish(planning_scene);

    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(washer);
    // planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    target_pose1.position.x = -1.21462;
    target_pose1.position.y = -0.199484;
    target_pose1.position.z = 0.187029;
    target_pose1.orientation.x = 0.55866;
    target_pose1.orientation.y = 0.0741221;
    target_pose1.orientation.z = -0.492944;
    target_pose1.orientation.w = 0.662882;
    move_group_interface.setPoseTarget(target_pose1);


    // joint_group_positions[0] = 8/180.0*M_PI;  // -1/6 turn in radians
    // move_group_interface.setJointValueTarget(joint_group_positions);
    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    if (success) {
    move_group_interface.execute(my_plan);
    } else {
    ROS_ERROR_STREAM_NAMED("tutorial", "Unable to plan");
    }
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


}