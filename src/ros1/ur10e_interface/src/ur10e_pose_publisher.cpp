#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>

class UR10ePosePublisher {
private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;

    // Subscribers
    ros::Subscriber joint_states_sub_;

    // Publishers
    ros::Publisher tool_pose_pub_;
    ros::Publisher base_odom_pub_;

    // Parameters
    std::string robot_base_frame_;
    std::string robot_tool_frame_;
    std::string robot_tcp_frame_;
    double publish_rate_;

    // Joint states for odometry calculation
    sensor_msgs::JointState last_joint_states_;
    bool joint_states_received_;
    ros::Time last_joint_states_time_;

public:
    UR10ePosePublisher() : nh_("~"), joint_states_received_(false) {
        // Load parameters
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        nh_.param<std::string>("robot_tool_frame", robot_tool_frame_, "tool0_controller");
        nh_.param<std::string>("robot_tcp_frame", robot_tcp_frame_, "tool0");
        nh_.param<double>("publish_rate", publish_rate_, 100.0);

        // Initialize subscribers and publishers
        joint_states_sub_ = nh_.subscribe("/joint_states", 1,
                                         &UR10ePosePublisher::jointStatesCallback, this);

        tool_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ur10e/tool0_pose", 1);
        base_odom_pub_ = nh_.advertise<nav_msgs/Odometry>("/ur10e/base_odom", 1);

        ROS_INFO("UR10e Pose Publisher initialized");
        ROS_INFO("Base frame: %s", robot_base_frame_.c_str());
        ROS_INFO("Tool frame: %s", robot_tool_frame_.c_str());
        ROS_INFO("TCP frame: %s", robot_tcp_frame_.c_str());
    }

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        last_joint_states_ = *msg;
        last_joint_states_time_ = ros::Time::now();
        joint_states_received_ = true;
    }

    void publishToolPose() {
        geometry_msgs::PoseWithCovarianceStamped pose_msg;

        try {
            // Get transform from base to tool frame
            tf::StampedTransform transform;
            tf_listener_.lookupTransform(robot_base_frame_, robot_tcp_frame_,
                                       ros::Time(0), transform);

            // Fill pose message
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = robot_base_frame_;

            pose_msg.pose.pose.position.x = transform.getOrigin().x();
            pose_msg.pose.pose.position.y = transform.getOrigin().y();
            pose_msg.pose.pose.position.z = transform.getOrigin().z();

            pose_msg.pose.pose.orientation.x = transform.getRotation().x();
            pose_msg.pose.pose.orientation.y = transform.getRotation().y();
            pose_msg.pose.pose.orientation.z = transform.getRotation().z();
            pose_msg.pose.pose.orientation.w = transform.getRotation().w();

            // Set covariance (small values for UR10e)
            pose_msg.pose.covariance = {
                1e-4, 0, 0, 0, 0, 0,
                0, 1e-4, 0, 0, 0, 0,
                0, 0, 1e-4, 0, 0, 0,
                0, 0, 0, 1e-3, 0, 0,
                0, 0, 0, 0, 1e-3, 0,
                0, 0, 0, 0, 0, 1e-3
            };

            tool_pose_pub_.publish(pose_msg);

        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF lookup error: %s", ex.what());
        }
    }

    void publishBaseOdometry() {
        if (!joint_states_received_) {
            return;
        }

        nav_msgs::Odometry odom_msg;

        // Calculate joint velocities for odometry
        if (last_joint_states_.velocity.size() == last_joint_states_.position.size()) {
            double total_velocity = 0.0;
            for (size_t i = 0; i < last_joint_states_.velocity.size(); ++i) {
                total_velocity += fabs(last_joint_states_.velocity[i]);
            }

            odom_msg.header.stamp = ros::Time::now();
            odom_msg.header.frame_id = robot_base_frame_;
            odom_msg.child_frame_id = robot_tcp_frame_;

            // Position (from current joint states via FK - simplified)
            try {
                tf::StampedTransform transform;
                tf_listener_.lookupTransform(robot_base_frame_, robot_tcp_frame_,
                                           ros::Time(0), transform);

                odom_msg.pose.pose.position.x = transform.getOrigin().x();
                odom_msg.pose.pose.position.y = transform.getOrigin().y();
                odom_msg.pose.pose.position.z = transform.getOrigin().z();

                odom_msg.pose.pose.orientation.x = transform.getRotation().x();
                odom_msg.pose.pose.orientation.y = transform.getRotation().y();
                odom_msg.pose.pose.orientation.z = transform.getRotation().z();
                odom_msg.pose.pose.orientation.w = transform.getRotation().w();

            } catch (tf::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "TF lookup error for odom: %s", ex.what());
                return;
            }

            // Velocity (simplified - using joint velocity sum as proxy)
            odom_msg.twist.twist.linear.x = total_velocity * 0.01;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = total_velocity * 0.01;
            odom_msg.twist.twist.angular.x = 0.0;
            odom_msg.twist.twist.angular.y = 0.0;
            odom_msg.twist.twist.angular.z = total_velocity * 0.05;

            // Set covariance
            odom_msg.pose.covariance = {
                1e-4, 0, 0, 0, 0, 0,
                0, 1e-4, 0, 0, 0, 0,
                0, 0, 1e-4, 0, 0, 0,
                0, 0, 0, 1e-3, 0, 0,
                0, 0, 0, 0, 1e-3, 0,
                0, 0, 0, 0, 0, 1e-3
            };

            odom_msg.twist.covariance = {
                1e-3, 0, 0, 0, 0, 0,
                0, 1e-3, 0, 0, 0, 0,
                0, 0, 1e-3, 0, 0, 0,
                0, 0, 0, 1e-2, 0, 0,
                0, 0, 0, 0, 1e-2, 0,
                0, 0, 0, 0, 0, 1e-2
            };

            base_odom_pub_.publish(odom_msg);
        }
    }

    void run() {
        ros::Rate rate(publish_rate_);

        while (ros::ok()) {
            // Publish UR10e pose and odometry for NavRL
            publishToolPose();
            publishBaseOdometry();

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur10e_pose_publisher");

    UR10ePosePublisher publisher;
    publisher.run();

    return 0;
}