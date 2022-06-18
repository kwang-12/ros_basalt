#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tbb/global_control.h>

#include "../include/device/rs_t265.h"
#include "../include/vi_estimator/vio_estimator.h"

basalt::Calibration<double> calib;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow_ptr;
basalt::VioEstimatorBase::Ptr vio;
// tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;

int main(int argc, char **argv)
{
    std::string node_name = "rs_t265_stream";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    ros::Publisher pub_vio_pose = n.advertise<geometry_msgs::PoseStamped>("basalt/pose", 100);
    ros::Publisher pub_vio_velocity = n.advertise<geometry_msgs::Vector3>("basalt/velocity", 100);

    geometry_msgs::PoseStamped pose_est;
    pose_est.header.frame_id = "world";
    pose_est.header.seq = 0;
    geometry_msgs::Vector3Stamped velocity_est;
    velocity_est.header.frame_id = "world";
    velocity_est.header.seq = 0;

    std::unique_ptr<tbb::global_control> tbb_global_control;
    tbb_global_control = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, 4);

    basalt::RsT265Device::Ptr t265_device;
    t265_device.reset(new basalt::RsT265Device(&n, false, 1, 100));
    t265_device->start();

    calib = *t265_device->exportCalibration();
    opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    t265_device->image_data_queue = &opt_flow_ptr->input_queue;

    vio = basalt::VioEstimatorFactory::getVioEstimator(vio_config, calib, basalt::constants::g, true, false);
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    t265_device->imu_data_queue = &vio->imu_data_queue;   // imu data goes to vio
    opt_flow_ptr->output_queue = &vio->vision_data_queue; // opt_flow output goes to vio

    // vio->out_vis_queue = &out_vis_queue; // TODO: pop and publish to ros
    vio->out_state_queue = &out_state_queue;

    basalt::PoseVelBiasState<double>::Ptr data;
    while (ros::ok())
    {
        out_state_queue.pop(data);
        if (data.get())
        {
            pose_est.header.stamp.fromNSec(data->t_ns);
            pose_est.pose.position.x = data->T_w_i.translation()[0];
            pose_est.pose.position.y = data->T_w_i.translation()[1];
            pose_est.pose.position.z = data->T_w_i.translation()[2];
            pose_est.pose.orientation.x = data->T_w_i.unit_quaternion().x();
            pose_est.pose.orientation.y = data->T_w_i.unit_quaternion().y();
            pose_est.pose.orientation.z = data->T_w_i.unit_quaternion().z();
            pose_est.pose.orientation.w = data->T_w_i.unit_quaternion().w();

            velocity_est.header.stamp = pose_est.header.stamp;
            velocity_est.vector.x = data->vel_w_i(0);
            velocity_est.vector.y = data->vel_w_i(1);
            velocity_est.vector.z = data->vel_w_i(2);

            pub_vio_pose.publish(pose_est);
            pub_vio_velocity.publish(velocity_est);
        }
        ros::spinOnce();
    }

    // ros::spin();

    t265_device->stop();
    ros::shutdown();
    return EXIT_SUCCESS;
}