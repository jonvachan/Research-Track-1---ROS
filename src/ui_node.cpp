#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Wait for the spawn service and call it
    ros::service::waitForService("/spawn");
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 2.0;
    spawn_srv.request.y = 3.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";

    if (client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 at (2.0, 3.0, 0.0).");
    } else {
        ROS_ERROR("Failed to call /spawn service.");
    }

    // Create publishers for turtle1 and turtle2
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    std::vector<ros::Publisher> pubs = {pub1, pub2};

    ros::Rate rate(10); // Loop rate
    while (ros::ok()) {
        int turtle_number;
        std::cout << "Enter the turtle number (1 or 2): ";
        std::cin >> turtle_number;

        if (turtle_number < 1 || turtle_number > 2) {
            std::cout << "Invalid turtle number. Please enter 1 or 2." << std::endl;
            continue;
        }

        geometry_msgs::Twist vel;
        std::cout << "Enter the linear velocity along x: ";
        std::cin >> vel.linear.x;
        std::cout << "Enter the linear velocity along y: ";
        std::cin >> vel.linear.y;
        std::cout << "Enter the angular velocity around z: ";
        std::cin >> vel.angular.z;

        // Publish velocity for 1 second
        ros::Time start_time = ros::Time::now();
        while ((ros::Time::now() - start_time).toSec() < 1.0) {
            pubs[turtle_number - 1].publish(vel);
            rate.sleep();
        }
    }
}
