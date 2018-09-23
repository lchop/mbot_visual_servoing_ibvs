#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mbot_visual_servoing_imagebased/GraspObjectAction.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_vs");

    mbot_visual_servoing_imagebased::GraspObjectGoal goal;
    actionlib::SimpleActionClient<mbot_visual_servoing_imagebased::GraspObjectAction> action_client("grasp_object", true);

    ROS_INFO("waiting for server: ");

    bool server_exists = action_client.waitForServer(); //wait forever

    ROS_INFO("connected to action server");
    std::string class_name = "muuug";
    while (ros::ok())
    {
        goal.class_name = class_name;

        action_client.sendGoal(goal);

        bool finished_before_timeout = action_client.waitForResult(); // wait forever
    }
}