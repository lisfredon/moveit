#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::vector<moveit_msgs::CollisionObject> collision_objects(3);

    // Table 1
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {0.2, 0.4, 0.4};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    // Table 2
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions = {0.4, 0.2, 0.4};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    // Objet à manipuler
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "panda_link0";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions = {0.02, 0.02, 0.2};
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(3.0).sleep();
    while (planning_scene_interface.getObjects().find("object") == planning_scene_interface.getObjects().end())
    {
        ROS_INFO("En attente que l'objet 'object' soit ajouté à la scène...");
        ros::Duration(0.5).sleep();
    }

    // Grasp
    std::vector<moveit_msgs::Grasp> grasps(1);
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI_2, -M_PI_4, -M_PI_2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);
    closedGripper(grasps[0].grasp_posture);

    move_group.setSupportSurfaceName("table1");

    bool success = (move_group.pick("object", grasps) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        ROS_WARN("Échec de la préhension!");

    ros::Duration(1.0).sleep();


    std::vector<moveit_msgs::PlaceLocation> place_location(1);
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion place_orientation;
    place_orientation.setRPY(0, 0, M_PI_2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(place_orientation);
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    move_group.setSupportSurfaceName("table2");

    success = (move_group.place("object", place_location) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        ROS_WARN("Échec du placement!");

    ros::shutdown();
    return 0;
}
