#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

class DistanceNode {
public:
    DistanceNode() {
        // Initialize node
        ros::NodeHandle nh;

        // Parameters
        distance_threshold = 1.5;  // Minimum distance between turtles
        boundary_threshold = 1.5; // Minimum distance to boundaries

        // Subscribers
        turtle1_sub = nh.subscribe("/turtle1/pose", 10, &DistanceNode::turtle1PoseCallback, this);
        turtle2_sub = nh.subscribe("/turtle2/pose", 10, &DistanceNode::turtle2PoseCallback, this);

        // Publishers
        distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
        turtle1_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        turtle2_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

        ROS_INFO("Distance Node Initialized");
    }

private:
    // Node parameters
    double distance_threshold;
    double boundary_threshold;

    // Pose data
    turtlesim::Pose turtle1_pose;
    turtlesim::Pose turtle2_pose;
    bool turtle1_pose_received = false;
    bool turtle2_pose_received = false;

    // Publishers and subscribers
    ros::Publisher distance_pub;
    ros::Publisher turtle1_vel_pub;
    ros::Publisher turtle2_vel_pub;
    ros::Subscriber turtle1_sub;
    ros::Subscriber turtle2_sub;

    // Callback for Turtle1's pose
    void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
        turtle1_pose = *msg;
        turtle1_pose_received = true;
        checkDistanceAndBoundaries();
    }

    // Callback for Turtle2's pose
    void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
        turtle2_pose = *msg;
        turtle2_pose_received = true;
        checkDistanceAndBoundaries();
    }

    // Check the distance and boundaries
    void checkDistanceAndBoundaries() {
        if (turtle1_pose_received && turtle2_pose_received) {
            // Calculate the Euclidean distance between the turtles
            double distance = std::sqrt(std::pow(turtle1_pose.x - turtle2_pose.x, 2) +
                                        std::pow(turtle1_pose.y - turtle2_pose.y, 2));

            // Publish the distance
            std_msgs::Float32 distance_msg;
            distance_msg.data = distance;
            distance_pub.publish(distance_msg);

            // Check if the distance is below the threshold
            if (distance < distance_threshold) {
                ROS_WARN("Turtles are too close! Stopping turtles.");
                stopTurtles();
                return;
            }

            // Check if Turtle1 is near the boundaries
            if (isNearBoundary(turtle1_pose)) {
                ROS_WARN("Turtle1 is near the boundary! Stopping turtles.");
                stopTurtles();
                return;
            }

            // Check if Turtle2 is near the boundaries
            if (isNearBoundary(turtle2_pose)) {
                ROS_WARN("Turtle2 is near the boundary! Stopping turtles.");
                stopTurtles();
                return;
            }
        }
    }

    // Helper function to check if a turtle is near the boundary
    bool isNearBoundary(const turtlesim::Pose& pose) {
        return (pose.x < boundary_threshold || pose.x > 11.0 - boundary_threshold ||
                pose.y < boundary_threshold || pose.y > 11.0 - boundary_threshold);
    }

    // Stop turtles by publishing zero velocity
    void stopTurtles() {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.linear.y = 0.0;
        turtle1_vel_pub.publish(stop_msg);
        turtle2_vel_pub.publish(stop_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_checking");
    DistanceNode node;

    ros::spin();
    return 0;
}
