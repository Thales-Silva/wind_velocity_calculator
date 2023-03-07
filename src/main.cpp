#include <wind_velocity_calculator/wind_velocity_calculator.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "wind_velocity_calculator");
    ros::NodeHandle nh;

    WindVelocityCalculator wvc(nh);

    ros::spin();

    return 0;
}
