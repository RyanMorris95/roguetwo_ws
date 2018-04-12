#include "sensor_fusion.h"
#include <geometry_msgs/Vector3.h>

SensorFusion::SensorFusion()
{
    dt = 0.01;
    float gps_std = pow(0.5, 2);
    float imu_std = pow(0.0007199025610000001 / 9.81, 2);  // the covariance given by the imu is in g's

    MatrixXf A(2, 2);
    A << 1, dt,
         0, 1;

    MatrixXf H(2, 2);
    H << 1, 0,
         0, 0;

    MatrixXf Q(2, 2);
    Q << imu_std, 0,
        0, imu_std;

    MatrixXf R(2, 2);
    R << gps_std, 0,
         0, gps_std;

    VectorXf X0(2);
    X0 << 0.0, 0.0;

    MatrixXf P0(2, 2);
    P0 << 0.0, 0.0,
          0.0, 0.0;

    MatrixXf B(2, 1);
    B << pow(0.5*dt, 2), dt;

    x_kalman = KalmanFilter();
    x_kalman.set_fixed(A, H, Q, R, B);
    x_kalman.set_initial(X0, P0);

    y_kalman = KalmanFilter();
    y_kalman.set_fixed(A, H, Q, R, B);
    y_kalman.set_initial(X0, P0);

    prev_seconds = 0;
}

void SensorFusion::predict(const sensor_msgs::Imu imu_msg)
{
    
    geometry_msgs::Vector3 accelerations = imu_msg.linear_acceleration;
    float x_accel = accelerations.x / 9.81;  // convert g to m/s^2
    float y_accel = accelerations.y / 9.81;

    std::cout << "X Accel: " << x_accel << std::endl;
    std::cout << "Y Accel: " << y_accel << std::endl;

    VectorXf u(1, 1);
    u << x_accel;
    x_kalman.predict(u);

    u(0, 0) = y_accel;
    y_kalman.predict(u);
    
    this->publish();
}


void SensorFusion::update(const roguetwo_perception::SE2 se2_msg)
{
    float x = se2_msg.x;
    float y = se2_msg.y;
    curr_yaw = se2_msg.yaw;

    VectorXf Z(2, 1);
    Z << x, 0.0;
    x_kalman.update(Z);

    Z(0, 0) = y;
    Z(1, 0) = 0;
    y_kalman.update(Z);

    this->publish();
}


void SensorFusion::publish()
{
    roguetwo_perception::SE2 se2_msg = roguetwo_perception::SE2();
    se2_msg.x = x_kalman.X0(0, 0);
    se2_msg.y = y_kalman.X0(0, 0);

    // std_msgs::Header header = std_msgs::Header();
    // header.stamp = ros::Time::now();
    // se2_msg.header = header;
    se2_filtered_pub.publish(se2_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_fusion");
    std::cout << "Created Sensor Fusion Node" << std::endl;
    SensorFusion sensor_fusion = SensorFusion();

    sensor_fusion.update_sub = sensor_fusion.nh.subscribe(
        "/se2_state",
        1,
        &SensorFusion::update,
        &sensor_fusion
    );

    sensor_fusion.predict_sub = sensor_fusion.nh.subscribe(
        "/imu/data",
        1,
        &SensorFusion::predict,
        &sensor_fusion
    );

    sensor_fusion.se2_filtered_pub = sensor_fusion.nh.advertise<roguetwo_perception::SE2>("/se2_state_filtered", 1);
    
    ros::spin();

    return 0;
}