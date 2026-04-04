#include "ee4308_drone/estimator.hpp"

namespace
{
    constexpr std::size_t ODOM_HISTORY_LENGTH = 30;
    constexpr double SONAR_TRUST_MAX_Z = 3.8;
    constexpr double SONAR_MAX_INNOVATION = 1.0;
    constexpr double MAX_ABS_VERTICAL_ACCEL = 1.5;

    void applyScalarCorrection(
        Eigen::Vector2d &X,
        Eigen::Matrix2d &P,
        const double measurement,
        const double variance)
    {
        if (!std::isfinite(measurement) || !std::isfinite(variance))
            return;

        Eigen::RowVector2d H;
        H << 1.0, 0.0;

        const double innovation = measurement - X(0);
        const double innovation_covariance = (H * P * H.transpose())(0, 0) + variance;
        if (!std::isfinite(innovation_covariance) || std::abs(innovation_covariance) < ee4308::THRES)
            return;

        const Eigen::Vector2d K = P * H.transpose() / innovation_covariance;
        X = X + K * innovation;
        P = P - K * H * P;
    }

    void applyAngularCorrection(
        Eigen::Vector2d &X,
        Eigen::Matrix2d &P,
        const double measurement,
        const double variance)
    {
        if (!std::isfinite(measurement) || !std::isfinite(variance))
            return;

        Eigen::RowVector2d H;
        H << 1.0, 0.0;

        const double innovation = ee4308::limitAngle(measurement - X(0));
        const double innovation_covariance = (H * P * H.transpose())(0, 0) + variance;
        if (!std::isfinite(innovation_covariance) || std::abs(innovation_covariance) < ee4308::THRES)
            return;

        const Eigen::Vector2d K = P * H.transpose() / innovation_covariance;
        X = X + K * innovation;
        X(0) = ee4308::limitAngle(X(0));
        P = P - K * H * P;
    }

    void applyScalarCorrection(
        Eigen::Vector3d &X,
        Eigen::Matrix3d &P,
        const double measurement,
        const double variance,
        const Eigen::RowVector3d &H)
    {
        if (!std::isfinite(measurement) || !std::isfinite(variance))
            return;

        const double innovation = measurement - (H * X)(0, 0);
        const double innovation_covariance = (H * P * H.transpose())(0, 0) + variance;
        if (!std::isfinite(innovation_covariance) || std::abs(innovation_covariance) < ee4308::THRES)
            return;

        const Eigen::Vector3d K = P * H.transpose() / innovation_covariance;
        X = X + K * innovation;
        P = P - K * H * P;
    }
}

namespace ee4308::drone
{
    Estimator::Estimator(
        const rclcpp::NodeOptions &options,
        const std::string &name = "estimator")
        : Node(name, options)
    {
        // parameters
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 10.0).as_double();
        this->var_imu_x_ = ee4308::getParameter<double>(this, "var_imu_x", 0.2).as_double();
        this->var_imu_y_ = ee4308::getParameter<double>(this, "var_imu_y", 0.2).as_double();
        this->var_imu_z_ = ee4308::getParameter<double>(this, "var_imu_z", 0.2).as_double();
        this->var_imu_a_ = ee4308::getParameter<double>(this, "var_imu_a", 0.2).as_double();
        this->var_gps_x_ = ee4308::getParameter<double>(this, "var_gps_x", 0.2).as_double();
        this->var_gps_y_ = ee4308::getParameter<double>(this, "var_gps_y", 0.2).as_double();
        this->var_gps_z_ = ee4308::getParameter<double>(this, "var_gps_z", 0.2).as_double();
        this->var_baro_ = ee4308::getParameter<double>(this, "var_baro", 0.2).as_double();
        this->var_sonar_ = ee4308::getParameter<double>(this, "var_sonar", 0.2).as_double();
        this->var_magnet_ = ee4308::getParameter<double>(this, "var_magnet", 0.2).as_double();
        this->verbose_ = ee4308::getParameter<bool>(this, "verbose", true).as_bool();
        this->frame_id_map_ = ee4308::getParameter<std::string>(this, "map_frame_id", "map").as_string();
        this->frame_id_drone_ = ee4308::getParameter<std::string>(this, "drone_frame_id", "drone/base_link").as_string();
        this->use_ground_truth_ = ee4308::getParameter<bool>(this, "use_ground_truth", false).as_bool();
        double initial_x = ee4308::getParameter<double>(this, "initial_x", -2.0).as_double();
        double initial_y = ee4308::getParameter<double>(this, "initial_y", -2.0).as_double();
        double initial_z = ee4308::getParameter<double>(this, "initial_z", 0.05).as_double();

        // topics
        this->pub_est_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::ServicesQoS());
        this->pub_est_path_ = this->create_publisher<nav_msgs::msg::Path>("odom_history", rclcpp::ServicesQoS());
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1); // use only the most recent
        this->sub_true_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "true_odom", qos, std::bind(&Estimator::callbackSubTrueOdom_, this, std::placeholders::_1)); // ground truth in sim.
        this->sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", qos, std::bind(&Estimator::callbackSubGPS_, this, std::placeholders::_1));
        this->sub_sonar_ = this->create_subscription<sensor_msgs::msg::LaserScan>( // gz has no sonar implementation. laserscan for quick hack.
            "sonar", qos, std::bind(&Estimator::callbackSubSonar_, this, std::placeholders::_1));
        this->sub_magnetic_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "magnetic", qos, std::bind(&Estimator::callbackSubMagnetic_, this, std::placeholders::_1));
        this->sub_baro_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "air_pressure", qos, std::bind(&Estimator::callbackSubBaro_, this, std::placeholders::_1));
        this->sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&Estimator::callbackSubIMU_, this, std::placeholders::_1));

        // states
        this->initial_position_ << initial_x, initial_y, initial_z;
        this->Xx_ << initial_x, 0;
        this->Xy_ << initial_y, 0;
        this->Xz_ << initial_z, 0.0, 0.0;
        this->Xa_ << 0, 0;
        this->Px_ = Eigen::Matrix2d::Constant(1e3),
        this->Py_ = Eigen::Matrix2d::Constant(1e3),
        this->Pz_ = Eigen::Matrix3d::Constant(1e3);
        this->Pa_ = Eigen::Matrix2d::Constant(1e3);
        this->initial_ECEF_ << NAN, NAN, NAN;
        this->Ygps_ << NAN, NAN, NAN;
        this->Ymagnet_ = NAN;
        this->Ybaro_ = NAN;
        this->Ysonar_ = NAN;
        this->est_path_.header.frame_id = this->frame_id_map_;

        this->last_predict_time_ = this->now().seconds();
        this->initialized_ecef_ = false;
        this->initialized_baro_ = false;
        this->initialized_magnetic_ = false;

        this->timer_ = this->create_timer(
            1s / this->frequency_,
            std::bind(&Estimator::callbackTimer, this));
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF_(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        const double a = RAD_EQUATOR;
        const double b = RAD_POLAR;
        const double b_sq_over_a_sq = (b * b) / (a * a);
        const double eccentricity_sq = 1.0 - b_sq_over_a_sq;
        const double prime_vertical_radius = a / std::sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat);

        Eigen::Vector3d ECEF;
        ECEF << (prime_vertical_radius + alt) * cos_lat * cos_lon,
            (prime_vertical_radius + alt) * cos_lat * sin_lon,
            (b_sq_over_a_sq * prime_vertical_radius + alt) * sin_lat;
        return ECEF;
    }

    void Estimator::callbackSubGPS_(const sensor_msgs::msg::NavSatFix msg)
    { // avoiding const & due to possibly long calcs.
        constexpr double DEG2RAD = M_PI / 180;
        double lat = msg.latitude * DEG2RAD;  
        double lon = msg.longitude * DEG2RAD; 
        double alt = msg.altitude;

        if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt))
        {
            Ygps_ << NAN, NAN, NAN;
            return;
        }

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        if (initialized_ecef_ == false)
        {
            initial_ECEF_ = getECEF_(sin_lat, cos_lat, sin_lon, cos_lon, alt);
            Ygps_ = initial_position_;
            initialized_ecef_ = true;
            return;
        }

        Eigen::Vector3d ECEF = getECEF_(sin_lat, cos_lat, sin_lon, cos_lon, alt);

        // After obtaining NED, and *rotating* to Gazebo's world frame,
        //      Store the measured x,y,z, in Ygps_.
        //      Required for terminal printing during demonstration.
        // The Gazebo world frame is in ENU instead of NED convention.
        // ==== make use of ====
        // Ygps_
        // initial_position_
        // initial_ECEF_
        // sin_lat, cost_lat, sin_lon, cos_lon, alt
        // var_gps_x_, var_gps_y_, var_gps_z_
        // Px_, Py_, Pz_
        // Xx_, Xy_, Xz_
        //
        // - other Eigen methods like .transpose().
        // - Possible to divide a VectorXd element-wise by a double by using the divide operator '/'.
        // - Matrix multiplication using the times operator '*'.
        // =========

        Eigen::Matrix3d R_e_n;
        R_e_n << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,
            -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,
            cos_lat, 0.0, -sin_lat;

        const Eigen::Vector3d ned = R_e_n.transpose() * (ECEF - initial_ECEF_);

        Eigen::Matrix3d R_m_n;
        R_m_n << 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0;
        Ygps_ = R_m_n * ned + initial_position_;

        applyScalarCorrection(Xx_, Px_, Ygps_(0), var_gps_x_);
        applyScalarCorrection(Xy_, Py_, Ygps_(1), var_gps_y_);
        Eigen::RowVector3d Hz;
        Hz << 1.0, 0.0, 0.0;
        applyScalarCorrection(Xz_, Pz_, Ygps_(2), var_gps_z_, Hz);
    }

    // ================================ Sonar sub callback / EKF Correction ========================================
    void Estimator::callbackSubSonar_(const sensor_msgs::msg::LaserScan msg)
    {
        // Store the measured sonar range in Ysonar_.
        //      Required for terminal printing during demonstration.
        // ==== make use of ====
        // msg.ranges[0]
        // Ysonar_
        // var_sonar_
        // Xz_
        // Pz_
        // .transpose()
        // =========

        Ysonar_ = msg.ranges.empty() ? NAN : msg.ranges[0];
        
        if (!std::isfinite(Ysonar_))
        { 
            // if out of range, write to Ysonar_, 
            //     but do not do the KF correction.
            return;
        }

        if (std::isfinite(msg.range_min) && Ysonar_ < msg.range_min)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TMP LOG sonar rejected: meas=%.3f below range_min=%.3f",
                Ysonar_,
                msg.range_min);
            return;
        }

        if (std::isfinite(msg.range_max) && Ysonar_ > msg.range_max)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TMP LOG sonar rejected: meas=%.3f above range_max=%.3f",
                Ysonar_,
                msg.range_max);
            return;
        }

        const double predicted_z = Xz_(0);
        const double sonar_innovation = Ysonar_ - predicted_z;
        const bool trust_low_altitude = predicted_z <= SONAR_TRUST_MAX_Z;
        const bool trust_consistent_measurement = std::abs(sonar_innovation) <= SONAR_MAX_INNOVATION;
        if (!trust_low_altitude || !trust_consistent_measurement)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TMP LOG sonar rejected: meas=%.3f est_z=%.3f innov=%.3f",
                Ysonar_,
                predicted_z,
                sonar_innovation);
            return;
        }

        Eigen::RowVector3d Hsonar;
        Hsonar << 1.0, 0.0, 0.0;
        applyScalarCorrection(Xz_, Pz_, Ysonar_, var_sonar_, Hsonar);
    }

    // ================================ Magnetic sub callback / EKF Correction ========================================
    void Estimator::callbackSubMagnetic_(const sensor_msgs::msg::MagneticField msg)
    {
        // Store the measured angle (world frame) in Ymagnet_.
        //      Required for terminal printing during demonstration.
        // Along the horizontal plane, the magnetic north in Gazebo points towards +x, when it should point to +y (even if world is configured to be ENU frame). It is a bug.
        // As the drone always starts pointing towards +x, there is no need to offset the calculation with an initial heading.
        // Magnetic force direction in drone's z-axis can be ignored.
        // The units are in Gauss (by Gazebo) instead of in Tesla (MagneticField message documentation).
        // ==== make use of ====
        // Ymagnet_
        // msg.magnetic_field.x // the magnetic force direction along drone's x-axis.
        // msg.magnetic_field.y // the magnetic force direction along drone's y-axis.
        // std::atan2()
        // Xa_
        // Pa_
        // var_magnet_
        // .transpose()
        // limitAngle()
        // =========

        const double mx = msg.magnetic_field.x;
        const double my = msg.magnetic_field.y;
        if (!std::isfinite(mx) || !std::isfinite(my) || std::hypot(mx, my) < ee4308::THRES)
        {
            Ymagnet_ = NAN;
            return;
        }

        Ymagnet_ = ee4308::limitAngle(std::atan2(-my, mx));
        initialized_magnetic_ = true;
        applyAngularCorrection(Xa_, Pa_, Ymagnet_, var_magnet_);
    }

    // ================================ Baro sub callback / EKF Correction ========================================
    void Estimator::callbackSubBaro_(const sensor_msgs::msg::FluidPressure msg)
    {
        // Store the measured barometer altitude in Ybaro_.
        //      Required for terminal printing during demonstration.
        // the fluid pressure is in pascal.
        // ==== make use of ====
        // Ybaro_ 
        // SEA_LEVEL_PA
        // msg.fluid_pressure
        // var_baro_
        // Pz_
        // Xz_
        // .transpose()
        // =========

        if (!std::isfinite(msg.fluid_pressure) || msg.fluid_pressure <= ee4308::THRES)
        {
            Ybaro_ = NAN;
            return;
        }

        Ybaro_ = 44330.0 * (1.0 - std::pow(msg.fluid_pressure / SEA_LEVEL_PA, 0.1903));

        if (!initialized_baro_)
        {
            initialized_baro_ = true;
            Xz_(2) = Ybaro_ - Xz_(0);
            Pz_(2, 2) = var_baro_;
            RCLCPP_INFO(
                this->get_logger(),
                "TMP LOG baro bias initialized: baro=%.3f est_z=%.3f bias=%.3f",
                Ybaro_, Xz_(0), Xz_(2));
        }

        // The barometer measures z plus a slowly varying bias, so the observation model is [1 0 1].
        Eigen::RowVector3d Hbaro;
        Hbaro << 1.0, 0.0, 1.0;
        applyScalarCorrection(Xz_, Pz_, Ybaro_, var_baro_, Hbaro);
    }

    // ================================ IMU sub callback / EKF Prediction ========================================
    void Estimator::callbackSubIMU_(const sensor_msgs::msg::Imu msg)
    {
        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();

        if (dt < ee4308::THRES)
            return;

        // NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!
        // Store the states in Xx_, Xy_, Xz_, and Xa_ for terminal printing.
        // Store the covariances in Px_, Py_, Pz_, and Pa_ for terminal printing.
        // ==== make use of ====
        // msg.linear_acceleration
        // msg.angular_velocity
        // GRAVITY
        // var_imu_x_, var_imu_y_, var_imu_z_, var_imu_a_
        // Xx_, Xy_, Xz_, Xa_
        // Px_, Py_, Pz_, Pa_
        // dt
        // std::cos(), std::sin()
        // =========
        Eigen::Matrix2d F;
        F << 1.0, dt,
             0.0, 1.0;

        Eigen::Vector2d W;
        W << 0.5 * dt * dt,
             dt;

        const double yaw = Xa_(0);
        const double ux = msg.linear_acceleration.x;
        const double uy = msg.linear_acceleration.y;
        const double raw_az = msg.linear_acceleration.z - GRAVITY;
        const double az = std::clamp(raw_az, -MAX_ABS_VERTICAL_ACCEL, MAX_ABS_VERTICAL_ACCEL);
        const double uyaw = msg.angular_velocity.z;

        if (std::abs(raw_az - az) > ee4308::THRES)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "TMP LOG imu z accel clamped: raw=%.3f clipped=%.3f",
                raw_az,
                az);
        }

        const double ax = std::cos(yaw) * ux - std::sin(yaw) * uy;
        const double ay = std::sin(yaw) * ux + std::cos(yaw) * uy;

        Eigen::Matrix2d Qxy;
        Qxy << var_imu_x_, 0.0,
            0.0, var_imu_y_;

        Eigen::Matrix<double, 2, 2> Wx;
        Wx << 0.5 * dt * dt * std::cos(yaw), -0.5 * dt * dt * std::sin(yaw),
            dt * std::cos(yaw), -dt * std::sin(yaw);

        Eigen::Matrix<double, 2, 2> Wy;
        Wy << 0.5 * dt * dt * std::sin(yaw), 0.5 * dt * dt * std::cos(yaw),
            dt * std::sin(yaw), dt * std::cos(yaw);

        Xx_ = F * Xx_ + W * ax;
        Px_ = F * Px_ * F.transpose() + Wx * Qxy * Wx.transpose();

        Xy_ = F * Xy_ + W * ay;
        Py_ = F * Py_ * F.transpose() + Wy * Qxy * Wy.transpose();

        Eigen::Matrix3d Fz = Eigen::Matrix3d::Identity();
        Fz(0, 1) = dt;

        Eigen::Vector3d Wz;
        Wz << 0.5 * dt * dt,
              dt,
              0.0;

        Xz_ = Fz * Xz_ + Wz * az;
        Pz_ = Fz * Pz_ * Fz.transpose() + Wz * var_imu_z_ * Wz.transpose();

        Eigen::Matrix2d Fa;
        Fa << 1.0, 0.0,
            0.0, 0.0;

        Eigen::Vector2d Wa;
        Wa << dt,
            1.0;

        Xa_ = Fa * Xa_ + Wa * uyaw;
        Xa_(0) = ee4308::limitAngle(Xa_(0));
        Pa_ = Fa * Pa_ * Fa.transpose() + Wa * var_imu_a_ * Wa.transpose();
    }

    void Estimator::callbackSubTrueOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->true_odom_ = msg;
    }

    void Estimator::publishOdomAndHistory_(const nav_msgs::msg::Odometry &odom)
    {
        this->pub_est_odom_->publish(odom);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.header.frame_id = odom.header.frame_id.empty() ? this->frame_id_map_ : odom.header.frame_id;
        pose_stamped.pose = odom.pose.pose;

        this->est_path_.header = pose_stamped.header;
        this->est_path_.poses.push_back(pose_stamped);
        if (this->est_path_.poses.size() > ODOM_HISTORY_LENGTH)
        {
            this->est_path_.poses.erase(
                this->est_path_.poses.begin(),
                this->est_path_.poses.begin() + (this->est_path_.poses.size() - ODOM_HISTORY_LENGTH));
        }
        this->pub_est_path_->publish(this->est_path_);
    }

    void Estimator::callbackTimer()
    {
        // ======= Publish and Verbose Ground Truth =======
        if (this->use_ground_truth_)
        {
            this->publishOdomAndHistory_(this->true_odom_);

            if (this->verbose_)
            {
                double t = this->now().seconds();
                std::stringstream ss;
                ss << std::fixed;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "TruPose" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.x << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.y << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.z << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::getYawFromQuaternion(true_odom_.pose.pose.orientation)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "TruTwis" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.x << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.y << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.z << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.angular.z
                   << std::endl;
                std::cout << ss.str() << std::endl;
            }
        }
        // ======= Publish and Verbose KF =======
        else
        {
            // you can extend this to include velocities if you want, but the topic name may have to change from pose to something else.
            // odom is already taken.
            nav_msgs::msg::Odometry odom;

            odom.header.stamp = this->now();
            odom.child_frame_id = "";     //; std::string(this->get_namespace()) + "/base_footprint";
            odom.header.frame_id = "map"; //; std::string(this->get_namespace()) + "/odom";

            odom.pose.pose.position.x = Xx_[0];
            odom.pose.pose.position.y = Xy_[0];
            odom.pose.pose.position.z = Xz_[0];
            getQuaternionFromYaw(Xa_[0], odom.pose.pose.orientation);
            odom.pose.covariance[0] = Px_(0, 0);
            odom.pose.covariance[7] = Py_(0, 0);
            odom.pose.covariance[14] = Pz_(0, 0);
            odom.pose.covariance[35] = Pa_(0, 0);

            odom.twist.twist.linear.x = Xx_[1];
            odom.twist.twist.linear.y = Xy_[1];
            odom.twist.twist.linear.z = Xz_[1];
            odom.twist.twist.angular.z = Xa_[1];
            odom.twist.covariance[0] = Px_(1, 1);
            odom.twist.covariance[7] = Py_(1, 1);
            odom.twist.covariance[14] = Pz_(1, 1);
            odom.twist.covariance[35] = Pa_(1, 1);

            this->publishOdomAndHistory_(odom);

            if (verbose_)
            {
                double t = this->now().seconds();
                std::stringstream ss;
                ss << std::fixed;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Pose" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xx_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xy_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(Xa_(0))
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Twist" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xx_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xy_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xa_(1)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "ErrPose" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.x - Xx_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.y - Xy_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.z - Xz_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(ee4308::getYawFromQuaternion(true_odom_.pose.pose.orientation) - Xa_(0))
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "ErrTwis" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.x - Xx_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.y - Xy_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.z - Xz_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.angular.z - Xa_(1)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "GPS" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(2) << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Baro"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ybaro_ << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "BBias"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(2) << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Sonar"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ysonar_ << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Magnt"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ymagnet_
                   << std::endl;
                std::cout << ss.str() << std::endl;
            }

            const bool has_true_odom =
                true_odom_.header.stamp.sec != 0 || true_odom_.header.stamp.nanosec != 0;
            if (has_true_odom)
            {
                // TMP LOG: keep a compact z/baro summary for parameter tuning and video review.
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "TMP LOG err_xyz=(%.3f, %.3f, %.3f) | z_est=%.3f vz_est=%.3f bias=%.3f | gps_z=%.3f sonar=%.3f baro=%.3f",
                    true_odom_.pose.pose.position.x - Xx_(0),
                    true_odom_.pose.pose.position.y - Xy_(0),
                    true_odom_.pose.pose.position.z - Xz_(0),
                    Xz_(0),
                    Xz_(1),
                    Xz_(2),
                    Ygps_(2),
                    Ysonar_,
                    Ybaro_);
            }
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Estimator);
