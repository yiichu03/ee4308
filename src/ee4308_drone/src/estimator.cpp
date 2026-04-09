#include "ee4308_drone/estimator.hpp"

namespace
{
    constexpr std::size_t ODOM_HISTORY_LENGTH = 30;
    constexpr double SONAR_TRUST_MAX_Z = 3.8;
    constexpr double SONAR_MAX_INNOVATION = 1.0;
    constexpr double MAX_ABS_VERTICAL_ACCEL = 1.5;

    template <typename MatrixType>
    void symmetrizeCovariance(MatrixType &P)
    {
        P = 0.5 * (P + P.transpose());
    }

    void applyScalarCorrection(
        Eigen::Vector2d &X,
        Eigen::Matrix2d &P,
        const double measurement,
        const double variance,
        const Eigen::RowVector2d &H)
    {
        if (!std::isfinite(measurement) || !std::isfinite(variance))
            return;

        const double innovation = measurement - (H * X)(0, 0);
        const double innovation_covariance = (H * P * H.transpose())(0, 0) + variance;
        if (!std::isfinite(innovation_covariance) || std::abs(innovation_covariance) < ee4308::THRES)
            return;

        const Eigen::Matrix2d P_prior = P;
        const Eigen::Vector2d K = P * H.transpose() / innovation_covariance;
        X = X + K * innovation;
        const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        const Eigen::Matrix2d KH = K * H;
        P = (I - KH) * P_prior * (I - KH).transpose() + K * variance * K.transpose();
        symmetrizeCovariance(P);
    }

    void applyScalarCorrection(
        Eigen::Vector2d &X,
        Eigen::Matrix2d &P,
        const double measurement,
        const double variance)
    {
        Eigen::RowVector2d H;
        H << 1.0, 0.0;
        applyScalarCorrection(X, P, measurement, variance, H);
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

        const Eigen::Matrix2d P_prior = P;
        const Eigen::Vector2d K = P * H.transpose() / innovation_covariance;
        X = X + K * innovation;
        X(0) = ee4308::limitAngle(X(0));
        const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        const Eigen::Matrix2d KH = K * H;
        P = (I - KH) * P_prior * (I - KH).transpose() + K * variance * K.transpose();
        symmetrizeCovariance(P);
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

        const Eigen::Matrix3d P_prior = P;
        const Eigen::Vector3d K = P * H.transpose() / innovation_covariance;
        X = X + K * innovation;
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d KH = K * H;
        P = (I - KH) * P_prior * (I - KH).transpose() + K * variance * K.transpose();
        symmetrizeCovariance(P);
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
        this->gps_forward_compensation_enable_ = ee4308::getParameter<bool>(this, "gps_forward_compensation_enable", true).as_bool();
        this->gps_forward_compensation_max_dt_ = ee4308::getParameter<double>(this, "gps_forward_compensation_max_dt", 0.5).as_double();
        this->gps_velocity_alpha_ = ee4308::getParameter<double>(this, "gps_velocity_alpha", 0.6).as_double();
        this->gps_velocity_variance_scale_ = ee4308::getParameter<double>(this, "gps_velocity_variance_scale", 1.0).as_double();
        this->gps_velocity_min_variance_ = ee4308::getParameter<double>(this, "gps_velocity_min_variance", 0.4).as_double();
        this->gps_velocity_max_innovation_ = ee4308::getParameter<double>(this, "gps_velocity_max_innovation", 1.5).as_double();
        this->gps_velocity_min_dt_ = ee4308::getParameter<double>(this, "gps_velocity_min_dt", 0.2).as_double();
        this->gps_velocity_max_dt_ = ee4308::getParameter<double>(this, "gps_velocity_max_dt", 2.0).as_double();
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
        this->sub_sonar_ = this->create_subscription<sensor_msgs::msg::LaserScan>( // In this project setup, Gazebo publishes the downward range reading as LaserScan, so it is used for the sonar correction.
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
        this->Px_ = Eigen::Matrix2d::Constant(1e3);
        this->Py_ = Eigen::Matrix2d::Constant(1e3);
        this->Pz_ = Eigen::Matrix3d::Constant(1e3);
        this->Pa_ = Eigen::Matrix2d::Constant(1e3);
        this->initial_ECEF_ << NAN, NAN, NAN;
        this->Ygps_ << NAN, NAN, NAN;
        this->last_gps_position_ << NAN, NAN;
        this->filtered_gps_velocity_.setZero();
        this->Ymagnet_ = NAN;
        this->Ybaro_ = NAN;
        this->Ysonar_ = NAN;
        this->est_path_.header.frame_id = this->frame_id_map_;

        this->last_predict_time_ = this->now().seconds();
        this->latest_state_stamp_ = this->now();
        this->initialized_ecef_ = false;
        this->initialized_baro_ = false;
        this->has_last_gps_measurement_ = false;
        this->initialized_gps_velocity_ = false;

        this->timer_ = this->create_timer(
            1s / this->frequency_,
            std::bind(&Estimator::callbackTimer, this));
    }

    void Estimator::maybeApplyGPSVelocityCorrection_(const rclcpp::Time &stamp)
    {
        if (gps_velocity_variance_scale_ <= 0.0)
            return;

        const Eigen::Vector2d gps_xy = Ygps_.head<2>();
        if (!std::isfinite(gps_xy(0)) || !std::isfinite(gps_xy(1)))
            return;

        if (!has_last_gps_measurement_)
        {
            last_gps_position_ = gps_xy;
            last_gps_stamp_ = stamp;
            has_last_gps_measurement_ = true;
            return;
        }

        const double dt_gps = (stamp - last_gps_stamp_).seconds();
        const Eigen::Vector2d delta_xy = gps_xy - last_gps_position_;
        last_gps_position_ = gps_xy;
        last_gps_stamp_ = stamp;

        if (!std::isfinite(dt_gps) || dt_gps < gps_velocity_min_dt_ || dt_gps > gps_velocity_max_dt_)
        {
            initialized_gps_velocity_ = false;
            return;
        }

        const Eigen::Vector2d raw_gps_velocity = delta_xy / dt_gps;
        if (!std::isfinite(raw_gps_velocity(0)) || !std::isfinite(raw_gps_velocity(1)))
            return;

        if (!initialized_gps_velocity_)
        {
            filtered_gps_velocity_ = raw_gps_velocity;
            initialized_gps_velocity_ = true;
        }
        else
        {
            filtered_gps_velocity_ =
                gps_velocity_alpha_ * raw_gps_velocity +
                (1.0 - gps_velocity_alpha_) * filtered_gps_velocity_;
        }

        Eigen::RowVector2d Hvel;
        Hvel << 0.0, 1.0;

        const double vel_variance_x = std::max(
            gps_velocity_min_variance_,
            gps_velocity_variance_scale_ * 2.0 * var_gps_x_ / (dt_gps * dt_gps));
        const double vel_variance_y = std::max(
            gps_velocity_min_variance_,
            gps_velocity_variance_scale_ * 2.0 * var_gps_y_ / (dt_gps * dt_gps));

        const double vel_innov_x = filtered_gps_velocity_(0) - Xx_(1);
        const double vel_innov_y = filtered_gps_velocity_(1) - Xy_(1);

        if (std::abs(vel_innov_x) <= gps_velocity_max_innovation_)
        {
            applyScalarCorrection(Xx_, Px_, filtered_gps_velocity_(0), vel_variance_x, Hvel);
        }
        if (std::abs(vel_innov_y) <= gps_velocity_max_innovation_)
        {
            applyScalarCorrection(Xy_, Py_, filtered_gps_velocity_(1), vel_variance_y, Hvel);
        }
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
        const rclcpp::Time stamp(msg.header.stamp);
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
            last_gps_position_ = Ygps_.head<2>();
            last_gps_stamp_ = stamp;
            has_last_gps_measurement_ = true;
            latest_state_stamp_ = stamp;
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

        Eigen::Vector3d gps_correction_measurement = Ygps_;
        const double dt_lag = last_predict_time_ - stamp.seconds();
        if (gps_forward_compensation_enable_ &&
            std::isfinite(dt_lag) &&
            dt_lag > ee4308::THRES &&
            dt_lag < gps_forward_compensation_max_dt_)
        {
            gps_correction_measurement(0) += Xx_(1) * dt_lag;
            gps_correction_measurement(1) += Xy_(1) * dt_lag;
            gps_correction_measurement(2) += Xz_(1) * dt_lag;
        }
        applyScalarCorrection(Xx_, Px_, gps_correction_measurement(0), var_gps_x_);
        applyScalarCorrection(Xy_, Py_, gps_correction_measurement(1), var_gps_y_);
        maybeApplyGPSVelocityCorrection_(stamp);
        Eigen::RowVector3d Hz;
        Hz << 1.0, 0.0, 0.0;
        applyScalarCorrection(Xz_, Pz_, gps_correction_measurement(2), var_gps_z_, Hz);
        this->latest_state_stamp_ = stamp;
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
            return;
        }

        if (std::isfinite(msg.range_max) && Ysonar_ > msg.range_max)
        {
            return;
        }

        const double predicted_z = Xz_(0);
        const double sonar_innovation = Ysonar_ - predicted_z;
        const bool trust_low_altitude = predicted_z <= SONAR_TRUST_MAX_Z;
        const bool trust_consistent_measurement = std::abs(sonar_innovation) <= SONAR_MAX_INNOVATION;
        if (!trust_low_altitude || !trust_consistent_measurement)
        {
            return;
        }

        Eigen::RowVector3d Hsonar;
        Hsonar << 1.0, 0.0, 0.0;
        applyScalarCorrection(Xz_, Pz_, Ysonar_, var_sonar_, Hsonar);
        this->latest_state_stamp_ = rclcpp::Time(msg.header.stamp);
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
        applyAngularCorrection(Xa_, Pa_, Ymagnet_, var_magnet_);
        this->latest_state_stamp_ = rclcpp::Time(msg.header.stamp);
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
        }

        // The barometer measures z plus a slowly varying bias, so the observation model is [1 0 1].
        Eigen::RowVector3d Hbaro;
        Hbaro << 1.0, 0.0, 1.0;
        applyScalarCorrection(Xz_, Pz_, Ybaro_, var_baro_, Hbaro);
        this->latest_state_stamp_ = rclcpp::Time(msg.header.stamp);
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
        symmetrizeCovariance(Px_);

        Xy_ = F * Xy_ + W * ay;
        Py_ = F * Py_ * F.transpose() + Wy * Qxy * Wy.transpose();
        symmetrizeCovariance(Py_);

        Eigen::Matrix3d Fz = Eigen::Matrix3d::Identity();
        Fz(0, 1) = dt;

        Eigen::Vector3d Wz;
        Wz << 0.5 * dt * dt,
              dt,
              0.0;

        Xz_ = Fz * Xz_ + Wz * az;
        Pz_ = Fz * Pz_ * Fz.transpose() + Wz * var_imu_z_ * Wz.transpose();
        symmetrizeCovariance(Pz_);

        Eigen::Matrix2d Fa;
        Fa << 1.0, 0.0,
            0.0, 0.0;

        Eigen::Vector2d Wa;
        Wa << dt,
            1.0;

        Xa_ = Fa * Xa_ + Wa * uyaw;
        Xa_(0) = ee4308::limitAngle(Xa_(0));
        Pa_ = Fa * Pa_ * Fa.transpose() + Wa * var_imu_a_ * Wa.transpose();
        symmetrizeCovariance(Pa_);
        this->latest_state_stamp_ = tnow;
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

            odom.header.stamp = this->latest_state_stamp_;
            odom.child_frame_id = this->frame_id_drone_;
            odom.header.frame_id = this->frame_id_map_;

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

        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Estimator);
