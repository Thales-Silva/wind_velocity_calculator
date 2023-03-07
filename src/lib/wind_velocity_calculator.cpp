#include <wind_velocity_calculator/wind_velocity_calculator.hpp>

WindVelocityCalculator::WindVelocityCalculator(ros::NodeHandle &_nH) :
    _nh(_nH),
    current_gps_health(0)
{
    std::string temp;
    std::string node_name = ros::this_node::getName();

    if (!_nh.getParam(node_name + "/gps_health_topic", temp) || temp.empty())
        throw std::runtime_error(BOLD_RED"[ ERROR] gps_health_topic is incorrect."
                                 "Please, set the parameter in congig.yaml");
    gps_health_sub = _nh.subscribe<std_msgs::UInt8>(temp, 5, &WindVelocityCalculator::gpsHealthCallback, this);
    temp.clear();

    if (!_nh.getParam(node_name + "/drone_linear_velocity_topic", temp) || temp.empty())
        throw std::runtime_error(BOLD_RED"[ ERROR] drone_linear_velocity_topic is incorrect."
                                 "Please, set the parameter in congig.yaml");
    drone_linear_velocity_sub.subscribe(_nh,temp, 5);
    temp.clear();

    if (!_nh.getParam(node_name + "/drone_angular_velocity_topic", temp) || temp.empty())
        throw std::runtime_error(BOLD_RED"[ ERROR] drone_angular_velocity_topic is incorrect."
                                 "Please, set the parameter in congig.yaml");
    drone_angular_velocity_sub.subscribe(_nh, temp, 5);
    temp.clear();

    if (!_nh.getParam(node_name + "/drone_orientation_topic", temp) || temp.empty())
        throw std::runtime_error(BOLD_RED"[ ERROR] drone_orientation_topic is incorrect."
                                 "Please, set the parameter in congig.yaml");
    drone_orientation_sub.subscribe(_nh, temp, 5);
    temp.clear();

    if (!_nh.getParam(node_name + "/anemometer_relative_velocity_topic", temp) || temp.empty())
        throw std::runtime_error(BOLD_RED"[ ERROR] anemometer_relative_velocity_topic is incorrect."
                                 "Please, set the parameter in congig.yaml");
    anemometer_relative_velocity_sub.subscribe(_nh, temp, 5);
    temp.clear();

    if (!_nh.getParam(node_name + "/wind_velocity_topic", temp) || temp.empty())
        throw std::runtime_error(BOLD_RED"[ ERROR] wind_velocity_topic is incorrect."
                                 "Please, set the parameter in congig.yaml");
    wind_velocity_pub = _nh.advertise<geometry_msgs::Vector3Stamped>(temp, 5);

    std::string temp2;
    if (_nh.getParam(node_name + "/vehicle_base_link", temp) &&
        _nh.getParam(node_name + "/measurement_link", temp2) &&
        !temp.empty() &&
        !temp2.empty())
    {
        tf::TransformListener tf_listener;
        tf::StampedTransform tf;

        if (tf_listener.waitForTransform(temp, temp2, ros::Time::now(), ros::Duration(5)) )
        {
            std::cout << BOLD_GREEN << "[ INFO] Setting anemometer realtive pose by searching the tf tree." << std::endl;

            tf_listener.lookupTransform(temp, temp2, ros::Time(0), tf);
            tf::Vector3 p = tf.getOrigin();
            tf::Quaternion q = tf.getRotation();

            anemometer_position = Eigen::Vector3d({p[0],
                                                   p[1],
                                                   p[2]});

            anemometer_orientation = Eigen::Quaterniond({q.getW(),
                                                         q.getX(),
                                                         q.getY(),
                                                         q.getZ()}).toRotationMatrix();
        }
        else
        {
            std::cerr << BOLD_YELLOW << "[ WARN] Setting anemometer realtive pose using .yaml parameters." << std::endl;
            std::vector<double> temp_vec;
            if (!_nh.getParam(node_name + "/anemometer_position", temp_vec) && !temp_vec.empty())
                throw std::runtime_error(BOLD_RED"[ ERROR] anemometer_position is invalid."
                                         "Please, set the parameter in congig.yaml");
            anemometer_position = Eigen::Vector3d({temp_vec[0],
                                                   temp_vec[1],
                                                   temp_vec[2]});
            temp_vec.clear();
            if (!_nh.getParam(node_name + "/anemometer_orientation", temp_vec) && !temp_vec.empty())
                throw std::runtime_error(BOLD_RED"[ ERROR] anemometer_orientation is invalid."
                                         "Please, set the parameter in congig.yaml");

            // ZYX minimal representation of orientation using rotation matrix
            // From measurement_link to FLU vehicle_base_link
            anemometer_orientation = Eigen::AngleAxisd(temp_vec[0], Eigen::Vector3d::UnitZ())
                                   * Eigen::AngleAxisd(temp_vec[1], Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(temp_vec[2], Eigen::Vector3d::UnitX());
        }
    }
    else
        throw std::runtime_error(BOLD_RED"[ ERROR] vehicle_base_link is incorrect."
                                 "Please, set the parameter in congig.yaml");

    std::cout << BOLD_GREEN << "[ INFO] Wind velocity calculator is up." << std::endl;
}

void WindVelocityCalculator::gpsHealthCallback(const std_msgs::UInt8::ConstPtr &_msg)
{
    if (current_gps_health != _msg->data)
    {
        if (_msg->data < 3)
        {
            std::cerr << BOLD_YELLOW << "[ WARN] GPS health " << int(_msg->data) << " is poor (<3). "
                                        "Wind velocity measurements are only valid when the drone is steady." << std::endl;
            _sync_ptr_improved.reset();
            _sync_ptr.reset(new Sync1(MySyncPolicy1(10), drone_orientation_sub,
                                                         anemometer_relative_velocity_sub));
            _sync_ptr->registerCallback(boost::bind(&WindVelocityCalculator::updateCallback, this, _1, _2));
        }
        else
        {
            std::cerr << BOLD_YELLOW << "[ WARN] GPS health " << int(_msg->data) << " is good (>=3). "
                                      "Improved wind velocity measurments." << std::endl;
            _sync_ptr.reset();
            _sync_ptr_improved.reset(new Sync2(MySyncPolicy2(10), drone_linear_velocity_sub,
                                                                  drone_angular_velocity_sub,
                                                                  drone_orientation_sub,
                                                                  anemometer_relative_velocity_sub));
            _sync_ptr_improved->registerCallback(boost::bind(&WindVelocityCalculator::updateCallback, this, _1, _2, _3, _4));
        }
    }
    current_gps_health = _msg->data;
}

void WindVelocityCalculator::updateCallback(const geometry_msgs::QuaternionStamped::ConstPtr &_orientation,
                                            const geometry_msgs::Vector3Stamped::ConstPtr &_anemometer_relative_vel)
{
    /*From FLY body coordinate system to ENU inertial coordinates*/
    Eigen::Matrix3d drone_orientation = Eigen::Quaterniond({_orientation->quaternion.w,
                                                            _orientation->quaternion.x,
                                                            _orientation->quaternion.y,
                                                            _orientation->quaternion.z}).toRotationMatrix();

    /*wind velocity mapped in ENU coordinate system*/
    Eigen::Vector3d wind_vel = drone_orientation * anemometer_orientation * Eigen::Vector3d({_anemometer_relative_vel->vector.x,
                                                                                             _anemometer_relative_vel->vector.y,
                                                                                             _anemometer_relative_vel->vector.z});

    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.vector.x = wind_vel[0];
    msg.vector.y = wind_vel[1];
    msg.vector.z = wind_vel[2];
    wind_velocity_pub.publish(msg);
}

void WindVelocityCalculator::updateCallback(const geometry_msgs::Vector3Stamped::ConstPtr &_lin_vel,
                                            const geometry_msgs::Vector3Stamped::ConstPtr &_ang_vel,
                                            const geometry_msgs::QuaternionStamped::ConstPtr &_orientation,
                                            const geometry_msgs::Vector3Stamped::ConstPtr &_anemometer_relative_vel)
{
    /*From FLU body coordinate system to ENU inertial coordinates*/
    Eigen::Matrix3d drone_orientation = Eigen::Quaterniond({_orientation->quaternion.w,
                                                            _orientation->quaternion.x,
                                                            _orientation->quaternion.y,
                                                            _orientation->quaternion.z}).toRotationMatrix();

    /*lin_vel is in ENU inertial coordinates (DJI documentation)
     * drone_linear_velocity is in FLU body coordinates
     */
    Eigen::Vector3d drone_linear_velocity = drone_orientation.transpose() * Eigen::Vector3d({_lin_vel->vector.x,
                                                                                             _lin_vel->vector.y,
                                                                                             _lin_vel->vector.z});

    /*ang_vel is in FLU coordinates (DJI documentation)
     *drone_angular_velocity is in FLU coordinates
     */
    Eigen::Vector3d drone_angular_velocity = Eigen::Vector3d({_ang_vel->vector.x,
                                                              _ang_vel->vector.y,
                                                              _ang_vel->vector.z});

    /*anemometer_velocity is the vector sum of the vehicle velocity and a vector product between the
     *body angular velocity with the sensor position relative to the body
     *anemometer_velocity is in FLU coordinates
     */
    Eigen::Vector3d anemometer_velocity = drone_linear_velocity + hatOperator(drone_angular_velocity) * anemometer_position;

    /*wind velocity mapped in ENU coordinate system*/
    Eigen::Vector3d wind_vel = drone_orientation * (anemometer_velocity - anemometer_orientation * Eigen::Vector3d({_anemometer_relative_vel->vector.x,
                                                                                                                    _anemometer_relative_vel->vector.y,
                                                                                                                    _anemometer_relative_vel->vector.z}));

    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.vector.x = wind_vel[0];
    msg.vector.y = wind_vel[1];
    msg.vector.z = wind_vel[2];
    wind_velocity_pub.publish(msg);
}

WindVelocityCalculator::~WindVelocityCalculator()
{
    std::cerr << BOLD_YELLOW << "[ WARN] Exiting wind velocity calculator!" << std::endl;
    std::cerr << BOLD_RED << std::endl;
}
