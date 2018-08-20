#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int32.h"

#include "std_msgs/String.h"

#include <iostream>
#include <bits/stdc++.h>
#include <unistd.h>
using namespace std;

ros::Publisher vaccum_pub;
int sensor_state = 0;
float target_x=0;
float target_y=0;
float target_z=0;


float image_x=0;
float image_y=0;
float image_z=0;
int vacuum_feedback=0;    // Feedback from pressure switch

void IOStatesCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int x1 = msg->data;
    sensor_state = x1;
    //ROS_INFO("Sensor State  : %f",sensor_state);

}



void ImagePoseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float temp_x=  msg->linear.x;
    float temp_y=  msg->linear.y;
    float temp_z=  msg->linear.z;

    image_x     = temp_x;
    image_y     = temp_y;
    image_z     = temp_z;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Subscriber number_subscriber  = node_handle.subscribe("/io_pins_status",10,IOStatesCallback);
    ros::Subscriber number_subscriber2 = node_handle.subscribe("/target_points",10,ImagePoseCallback);
    //  ros::Rate loop_rate(10);
    /* This sleep is ONLY to allow Rviz to come up */
    sleep(5.0);

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name
    // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface group("arm");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;



    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    vaccum_pub=node_handle.advertise<std_msgs::Int32>("/vaccum",1000);



    std_msgs::StringPtr str(new std_msgs::String);

    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.




    // Now, we call the planner to compute the plan
    // and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation  = tf::createQuaternionMsgFromRollPitchYaw(3.13,0.01,1.57);   //  (3.13,0.01,2.35)1.  (-1.57,0,-1.57);    2.(-1.57,0,0)
    target_pose1.position.x = 0.024;
    target_pose1.position.y = -0.502;
    target_pose1.position.z = 0.703;


    geometry_msgs::Pose target_pose2;
    target_pose2.orientation  = tf::createQuaternionMsgFromRollPitchYaw(3.13,0.01,3.13);   //1.  (-1.57,0,-1.57);    2.(-1.57,0,0)
    target_pose2.position.x = 0.502;
    target_pose2.position.y = -0.002;
    target_pose2.position.z = 0.703;

    geometry_msgs::Pose target_pose3;
    target_pose3.orientation  = tf::createQuaternionMsgFromRollPitchYaw(3.13,0.01,3.13);   //1.  (-1.57,0,-1.57);    2.(-1.57,0,0)
    target_pose3.position.x = 0.502;
    target_pose3.position.y = -0.002;
    target_pose3.position.z = 0.390;





    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    group_variable_values[0] = 0;
    group.setJointValueTarget(group_variable_values);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = static_cast<bool>(group.plan(my_plan));

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    group.move();
    sleep(5.0);

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // Now that we have a plan we can visualize it in Rviz.  This is not
    // necessary because the group.plan() call we made above did this
    // automatically.  But explicitly publishing plans is useful in cases that we
    // want to visualize a previously created plan.





    while(ros::ok())

    {
        target_x  = image_x ;          // 0.02 manual offset
        target_y  = image_y ;   //  0.06 manual offset
        target_z  = image_z + 0.32;   //   0.26 is end effector length and suction pad offset 0.01 and 0.05 is offset MANUAL




        std_msgs::Int32 msg1;
        msg1.data = 0;
        vaccum_pub.publish(msg1);
        ROS_INFO("Turn OFF Vaccum Confirmation");



        // Adding/Removing Objects and Attaching/Detaching Objects
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // First, we will define the collision object message.
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = group.getPlanningFrame();

        /* The id of the object is used to identify it. */
        collision_object.id = "box1";

        /* Define a box to add to the world. */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.55;
        primitive.dimensions[1] = 0.55;
        primitive.dimensions[2] = 0.1;

        /* A pose for the box (specified relative to frame_id) */
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x =  0;
        box_pose.position.y = 0;
        box_pose.position.z =  -0.075;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        // Now, let's add the collision object into the world
        ROS_INFO("Add an object into the world");
        planning_scene_interface.addCollisionObjects(collision_objects);

        /* Sleep so we have time to see the object in RViz */
        sleep(2.0);
        //ROS_INFO("Check 1");
        // Planning with collision detection can be slow.  Lets set the planning time
        // to be sure the planner has enough time to plan around the box.  10 seconds
        // should be plenty.
        group.setPlanningTime(20.0);
        group.allowReplanning(true);
        robot_state::RobotState start_state(*group.getCurrentState());
        group.setStartState(*group.getCurrentState());
        group.setPoseTarget(target_pose1);
        success = static_cast<bool>(group.plan(my_plan));
        ROS_INFO("Visualizing plan 1 (pose goal move around box) %s",
                 success?"":"FAILED");
        /* Sleep to give Rviz time to visualize the plan. */
        //   sleep(5.0);
        group.move();
        //    sleep(5.0);
        ROS_INFO("Target 1");



        geometry_msgs::Pose target_image;
        target_image.orientation  = tf::createQuaternionMsgFromRollPitchYaw(3.10,0.06,1.05);
        ROS_INFO("target_x: %f  target_y: %f  target_z: %f",target_x,target_y,target_z); //1.  (-1.57,0,-1.57);    2.(-1.57,0,0)
        target_image.position.x = target_x;
        target_image.position.y = target_y;
        target_image.position.z = target_z;//target_z - 0.26;



        group.setPoseTarget(target_image);
        success = static_cast<bool>(group.plan(my_plan));
        ROS_INFO("Visualizing plan target_image (pose goal) %s",success?"":"FAILED");
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(2.0);
        group.move();
        sleep(2.0);
        ROS_INFO("Target Image reached");
        msg1.data = 1;
        vaccum_pub.publish(msg1);
        ROS_INFO("Turn ON Vaccum");
        
        
        if(vacuum_feedback == 0)
        {
            ROS_INFO("Going down again");
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose2);

            geometry_msgs::Pose target_image_extn = target_image;

            target_image_extn.position.z = target_z -0.01;
            waypoints.push_back(target_image_extn);  // down
            group.setMaxVelocityScalingFactor(0.1);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            group.setPoseTarget(target_image_extn);
            success = static_cast<bool>(group.plan(my_plan));
            group.move();
            sleep(5.0);

            ROS_INFO("Extension point reached ");


        }
        
        




        group.setPoseTarget(target_pose1);
        success = static_cast<bool>(group.plan(my_plan));
        ROS_INFO("Visualizing plan 1 (pose goal move around box) %s",
                 success?"":"FAILED");
        group.move();
        sleep(1.0);

        ROS_INFO("Target 1 again");

        group.setPoseTarget(target_pose2);
        success = static_cast<bool>(group.plan(my_plan));
        ROS_INFO("Visualizing plan 2 (pose goal move around box) %s",
                 success?"":"FAILED");
        group.move();
        sleep(1.0);
        ROS_INFO("Target 2");


        group.setPoseTarget(target_pose3);
        success = static_cast<bool>(group.plan(my_plan));
        ROS_INFO("Visualizing plan 3 (pose goal move around box) %s",
                 success?"":"FAILED");
        group.move();
        sleep(1.0);
        ROS_INFO("Target 3,Packet dropped");





        group.setPoseTarget(target_pose2);
        success = static_cast<bool>(group.plan(my_plan));
        ROS_INFO("Visualizing plan 2 (pose goal move around box) %s",
                 success?"":"FAILED");
        group.move();
        sleep(1.0);
        ROS_INFO("Target 2 Again");



        ROS_INFO("Mission completed");

        // Now, let's remove the collision object from the world.
        ROS_INFO("Remove the object from the world");
        std::vector<std::string> object_ids;
        object_ids.push_back(collision_object.id);
        planning_scene_interface.removeCollisionObjects(object_ids);

        /* Sleep to give Rviz time to show the object is no longer there. */
        sleep(1.0);


    }
// END_TUTORIAL

    ros::shutdown();
    return 0;
}




