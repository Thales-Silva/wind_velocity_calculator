#ifndef WIND_VELOCITY_CALCULATOR_HPP
#define WIND_VELOCITY_CALCULATOR_HPP

#define BOLD_RED "\033[1;31m"
#define BOLD_GREEN "\033[1;32m"
#define BOLD_YELLOW "\033[1;33m"

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/UInt8.h>

#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <eigen3/Eigen/Dense>

#include <boost/bind.hpp>

#include <vector>

class WindVelocityCalculator
{
    private:
        ros::NodeHandle &_nh;
        ros::Subscriber gps_health_sub;
        ros::Publisher wind_velocity_pub;

        message_filters::Subscriber<geometry_msgs::Vector3Stamped> drone_linear_velocity_sub;
        message_filters::Subscriber<geometry_msgs::Vector3Stamped> drone_angular_velocity_sub;
        message_filters::Subscriber<geometry_msgs::QuaternionStamped> drone_orientation_sub;
        message_filters::Subscriber<geometry_msgs::Vector3Stamped> anemometer_relative_velocity_sub;

        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::QuaternionStamped,
                                                                geometry_msgs::Vector3Stamped> MySyncPolicy1;
        typedef message_filters::Synchronizer<MySyncPolicy1> Sync1;

        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped,
                                                                geometry_msgs::Vector3Stamped,
                                                                geometry_msgs::QuaternionStamped,
                                                                geometry_msgs::Vector3Stamped> MySyncPolicy2;
        typedef message_filters::Synchronizer<MySyncPolicy2> Sync2;

        boost::shared_ptr<Sync1> _sync_ptr;
        boost::shared_ptr<Sync2> _sync_ptr_improved;

        Eigen::Vector3d anemometer_position;
        Eigen::Matrix3d anemometer_orientation;
        u_int8_t current_gps_health;

    public:
        WindVelocityCalculator(ros::NodeHandle &_nH);
        ~WindVelocityCalculator();

        void gpsHealthCallback(const std_msgs::UInt8::ConstPtr &_msg);
        void updateCallback(const geometry_msgs::QuaternionStamped::ConstPtr &_orientation,
                            const geometry_msgs::Vector3Stamped::ConstPtr &_anemometer_relative_vel);
        void updateCallback(const geometry_msgs::Vector3Stamped::ConstPtr &_lin_vel,
                            const geometry_msgs::Vector3Stamped::ConstPtr &_ang_vel,
                            const geometry_msgs::QuaternionStamped::ConstPtr &_orientation,
                            const geometry_msgs::Vector3Stamped::ConstPtr &_anemometer_relative_vel);
        inline Eigen::Matrix3d hatOperator(Eigen::Vector3d v)
        {
            Eigen::Matrix3d v_hat;
            v_hat <<   0, -v(2),  v(1),
                    v(2),     0, -v(0),
                   -v(1),  v(0),     0;
            return v_hat;
        }
};

#endif
