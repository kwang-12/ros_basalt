/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros_basalt/device/rs_t265.h>

std::string get_date();

namespace basalt
{

    RsT265Device::RsT265Device(ros::NodeHandle *_n,
                               bool manual_exposure, int skip_frames,
                               int webp_quality, double exposure_value)
        : manual_exposure(manual_exposure),
          skip_frames(skip_frames),
          webp_quality(webp_quality)
    {
        // initialize ros related components
        n = _n;
        pub_t265_cam_img[0] = n->advertise<sensor_msgs::Image>("rs_t265/cam0", 100);
        pub_t265_cam_img[1] = n->advertise<sensor_msgs::Image>("rs_t265/cam1", 100);
        for (int i = 0; i < 2; i++)
        {
            t265_cam_img[i].header.frame_id = "t265";
            t265_cam_img[i].header.seq = 0;
            t265_cam_img[i].height = 800;
            t265_cam_img[i].width = 848;
            t265_cam_img[i].encoding = sensor_msgs::image_encodings::MONO8;
            t265_cam_img[i].is_bigendian = 0;
            t265_cam_img[i].step = 848 * sizeof(uint8_t);
        }

        pub_t265_aligned_imu = n->advertise<sensor_msgs::Imu>("rs_t265/imu", 100);
        t265_aligned_imu.header.frame_id = "t265";
        t265_aligned_imu.header.seq = 0;
        for (int i = 0; i < 9; i++)
        {
            t265_aligned_imu.orientation_covariance[i] = 0;
            t265_aligned_imu.angular_velocity_covariance[i] = 0;
            t265_aligned_imu.linear_acceleration_covariance[i] = 0;
        }
        t265_aligned_imu.orientation.x = 0;
        t265_aligned_imu.orientation.y = 0;
        t265_aligned_imu.orientation.z = 0;
        t265_aligned_imu.orientation.w = 1;
        t265_aligned_imu.angular_velocity.x = 0;
        t265_aligned_imu.angular_velocity.y = 0;
        t265_aligned_imu.angular_velocity.z = 0;
        t265_aligned_imu.linear_acceleration.x = 0;
        t265_aligned_imu.linear_acceleration.y = 0;
        t265_aligned_imu.linear_acceleration.z = 0;

        // initalize realsense t265 camera
        pipe = rs2::pipeline(context);
        config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
        // if (!manual_exposure)
        // {
        //     config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        // }
        if (context.query_devices().size() == 0)
        {
            ROS_INFO("Waiting for realsense T265 connection...");
            rs2::device_hub hub(context);
            hub.wait_for_device();
        }
        auto device = context.query_devices()[0];
        device.hardware_reset();
        ROS_INFO_STREAM("realsense device " << device.get_info(RS2_CAMERA_INFO_NAME) << " connected\n");

        sensor = device.query_sensors()[0];
        if (manual_exposure)
        {
            ROS_INFO("Enabling manual exposure control");
            sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            sensor.set_option(rs2_option::RS2_OPTION_EXPOSURE, exposure_value * 1000);
        }
    }

    void RsT265Device::start()
    {
        auto callback = [&](const rs2::frame &frame)
        {
            exportCalibration();

            if (auto fp = frame.as<rs2::motion_frame>())
            {
                auto motion = frame.as<rs2::motion_frame>();

                if (motion &&
                    motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
                    motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
                {
                    RsIMUData d;
                    d.timestamp = motion.get_timestamp();
                    d.data << motion.get_motion_data().x,
                        motion.get_motion_data().y,
                        motion.get_motion_data().z;

                    gyro_data_queue.emplace_back(d);
                }
                else if (motion &&
                         motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
                         motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
                {
                    RsIMUData d;
                    d.timestamp = motion.get_timestamp();
                    d.data << motion.get_motion_data().x,
                        motion.get_motion_data().y,
                        motion.get_motion_data().z;

                    if (!prev_accel_data.get())
                    {
                        prev_accel_data.reset(new RsIMUData(d));
                    }
                    else
                    {
                        // BASALT_ASSERT(d.timestamp > prev_accel_data->timestamp);
                        while (!gyro_data_queue.empty() &&
                               gyro_data_queue.front().timestamp < prev_accel_data->timestamp)
                        {
                            std::cout << "Skipping gyro data. Timestamp before the first accel "
                                         "measurement.";
                            gyro_data_queue.pop_front();
                        }

                        while (!gyro_data_queue.empty() &&
                               gyro_data_queue.front().timestamp < d.timestamp)
                        {
                            RsIMUData gyro_data = gyro_data_queue.front();
                            gyro_data_queue.pop_front();

                            double w0 = (d.timestamp - gyro_data.timestamp) /
                                        (d.timestamp - prev_accel_data->timestamp);

                            double w1 = (gyro_data.timestamp - prev_accel_data->timestamp) /
                                        (d.timestamp - prev_accel_data->timestamp);

                            Eigen::Vector3d accel_interpolated =
                                w0 * prev_accel_data->data + w1 * d.data;

                            basalt::ImuData<double>::Ptr data;
                            data.reset(new basalt::ImuData<double>);
                            data->t_ns = gyro_data.timestamp * 1e6;
                            data->accel = accel_interpolated;
                            data->gyro = gyro_data.data;

                            if (imu_data_queue)
                            {
                                imu_data_queue->push(data);
                            }

                            // publish to ros
                            t265_aligned_imu.header.stamp.fromNSec(data->t_ns);
                            t265_aligned_imu.header.seq += 1;
                            t265_aligned_imu.angular_velocity.x = gyro_data.data.x();
                            t265_aligned_imu.angular_velocity.y = gyro_data.data.y();
                            t265_aligned_imu.angular_velocity.z = gyro_data.data.z();
                            t265_aligned_imu.linear_acceleration.x = accel_interpolated.x();
                            t265_aligned_imu.linear_acceleration.y = accel_interpolated.y();
                            t265_aligned_imu.linear_acceleration.z = accel_interpolated.z();
                            pub_t265_aligned_imu.publish(t265_aligned_imu);
                        }

                        prev_accel_data.reset(new RsIMUData(d));
                    }
                }
            }
            else if (auto fs = frame.as<rs2::frameset>())
            {
                // BASALT_ASSERT(fs.size() == NUM_CAMS);

                std::vector<rs2::video_frame> vfs;
                for (int i = 0; i < NUM_CAMS; ++i)
                {
                    rs2::video_frame vf = fs[i].as<rs2::video_frame>();
                    if (!vf)
                    {
                        std::cout << "Weird Frame, skipping" << std::endl;
                        return;
                    }
                    vfs.push_back(vf);
                }

                // Callback is called for every new image, so in every other call, the
                // left frame is updated but the right frame is still from the previous
                // timestamp. So we only process framesets where both images are valid and
                // have the same timestamp.
                for (int i = 1; i < NUM_CAMS; ++i)
                {
                    if (vfs[0].get_timestamp() != vfs[i].get_timestamp())
                    {
                        return;
                    }
                }

                // skip frames if configured
                if (frame_counter++ % skip_frames != 0)
                {
                    return;
                }

                OpticalFlowInput::Ptr data(new OpticalFlowInput);
                data->img_data.resize(NUM_CAMS);

                for (int i = 0; i < NUM_CAMS; i++)
                {
                    const auto &vf = vfs[i];
                    data->t_ns = vf.get_timestamp() * 1e6;
                    t265_cam_img[i].header.stamp.fromNSec(data->t_ns);

                    data->img_data[i].exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-6;
                    data->img_data[i].img.reset(new basalt::ManagedImage<uint16_t>(vf.get_width(), vf.get_height()));
                    t265_cam_img[i].data.clear();
                    t265_cam_img[i].header.seq += 1;

                    const uint8_t *data_in = (const uint8_t *)vf.get_data();
                    uint16_t *data_out = data->img_data[i].img->ptr;
                    size_t full_size = vf.get_width() * vf.get_height();
                    for (size_t j = 0; j < full_size; j++)
                    {
                        int val = data_in[j];
                        val = val << 8;
                        data_out[j] = val;
                        t265_cam_img[i].data.push_back(data_in[j]);
                    }
                    pub_t265_cam_img[i].publish(t265_cam_img[i]);
                }
                last_img_data = data;
                if (image_data_queue)
                {
                    image_data_queue->push(data);
                }
            }
            // else if (auto pf = frame.as<rs2::pose_frame>())
            // {
            //     auto data = pf.get_pose_data();

            //     RsPoseData pdata;
            //     pdata.t_ns = pf.get_timestamp() * 1e6;

            //     Eigen::Vector3d trans(data.translation.x, data.translation.y,
            //                           data.translation.z);
            //     Eigen::Quaterniond quat(data.rotation.w, data.rotation.x, data.rotation.y,
            //                             data.rotation.z);

            //     pdata.data = Sophus::SE3d(quat, trans);

            //     if (pose_data_queue)
            //         pose_data_queue->push(pdata);
            // }
        };

        profile = pipe.start(config, callback);
    }

    void RsT265Device::stop()
    {
        if (image_data_queue)
        {
            image_data_queue->push(nullptr);
        }
        if (imu_data_queue)
        {
            imu_data_queue->push(nullptr);
        }
    }

    bool RsT265Device::setExposure(double exposure)
    {
        if (!manual_exposure)
            return false;

        sensor.set_option(rs2_option::RS2_OPTION_EXPOSURE, exposure * 1000);
        return true;
    }

    void RsT265Device::setSkipFrames(int skip)
    {
        skip_frames = skip;
    }

    void RsT265Device::setWebpQuality(int quality)
    {
        webp_quality = quality;
    }

    std::shared_ptr<basalt::Calibration<double>> RsT265Device::exportCalibration()
    {
        using Scalar = double;

        if (calib.get())
            return calib;

        calib.reset(new basalt::Calibration<Scalar>);
        calib->imu_update_rate = IMU_RATE;

        auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL);
        auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO);
        auto cam0_stream = profile.get_stream(RS2_STREAM_FISHEYE, 1);
        auto cam1_stream = profile.get_stream(RS2_STREAM_FISHEYE, 2);

        // get gyro extrinsics
        if (auto gyro = gyro_stream.as<rs2::motion_stream_profile>())
        {
            rs2_motion_device_intrinsic intrinsics = gyro.get_motion_intrinsics();

            Eigen::Matrix<Scalar, 12, 1> gyro_bias_full;
            gyro_bias_full << intrinsics.data[0][3], intrinsics.data[1][3],
                intrinsics.data[2][3], intrinsics.data[0][0] - 1.0,
                intrinsics.data[1][0], intrinsics.data[2][0], intrinsics.data[0][1],
                intrinsics.data[1][1] - 1.0, intrinsics.data[2][1],
                intrinsics.data[0][2], intrinsics.data[1][2],
                intrinsics.data[2][2] - 1.0;
            basalt::CalibGyroBias<Scalar> gyro_bias;
            gyro_bias.getParam() = gyro_bias_full;
            calib->calib_gyro_bias = gyro_bias;

            // std::cout << "Gyro Bias\n" << gyro_bias_full << std::endl;

            calib->gyro_noise_std = Eigen::Vector3d(intrinsics.noise_variances[0],
                                                    intrinsics.noise_variances[1],
                                                    intrinsics.noise_variances[2])
                                        .cwiseSqrt();

            calib->gyro_bias_std = Eigen::Vector3d(intrinsics.bias_variances[0],
                                                   intrinsics.bias_variances[1],
                                                   intrinsics.bias_variances[2])
                                       .cwiseSqrt();

            // std::cout << "Gyro noise var: " << intrinsics.noise_variances[0]
            //          << " bias var: " << intrinsics.bias_variances[0] << std::endl;
        }
        else
        {
            std::abort();
        }

        // get accel extrinsics
        if (auto accel = accel_stream.as<rs2::motion_stream_profile>())
        {
            rs2_motion_device_intrinsic intrinsics = accel.get_motion_intrinsics();
            Eigen::Matrix<Scalar, 9, 1> accel_bias_full;
            accel_bias_full << intrinsics.data[0][3], intrinsics.data[1][3],
                intrinsics.data[2][3], intrinsics.data[0][0] - 1.0,
                intrinsics.data[1][0], intrinsics.data[2][0],
                intrinsics.data[1][1] - 1.0, intrinsics.data[2][1],
                intrinsics.data[2][2] - 1.0;
            basalt::CalibAccelBias<Scalar> accel_bias;
            accel_bias.getParam() = accel_bias_full;
            calib->calib_accel_bias = accel_bias;

            // std::cout << "Gyro Bias\n" << accel_bias_full << std::endl;

            calib->accel_noise_std = Eigen::Vector3d(intrinsics.noise_variances[0],
                                                     intrinsics.noise_variances[1],
                                                     intrinsics.noise_variances[2])
                                         .cwiseSqrt();

            calib->accel_bias_std = Eigen::Vector3d(intrinsics.bias_variances[0],
                                                    intrinsics.bias_variances[1],
                                                    intrinsics.bias_variances[2])
                                        .cwiseSqrt();

            // std::cout << "Accel noise var: " << intrinsics.noise_variances[0]
            //          << " bias var: " << intrinsics.bias_variances[0] << std::endl;
        }
        else
        {
            std::abort();
        }

        // get camera ex-/intrinsics
        for (const auto &cam_stream : {cam0_stream, cam1_stream})
        {
            if (auto cam = cam_stream.as<rs2::video_stream_profile>())
            {
                // extrinsics
                rs2_extrinsics ex = cam.get_extrinsics_to(gyro_stream);
                Eigen::Matrix3f rot = Eigen::Map<Eigen::Matrix3f>(ex.rotation);
                Eigen::Vector3f trans = Eigen::Map<Eigen::Vector3f>(ex.translation);

                Eigen::Quaterniond q(rot.cast<double>());
                basalt::Calibration<Scalar>::SE3 T_i_c(q, trans.cast<double>());

                // std::cout << "T_i_c\n" << T_i_c.matrix() << std::endl;

                calib->T_i_c.push_back(T_i_c);

                // get resolution
                Eigen::Vector2i resolution;
                resolution << cam.width(), cam.height();
                calib->resolution.push_back(resolution);

                // intrinsics
                rs2_intrinsics intrinsics = cam.get_intrinsics();
                basalt::KannalaBrandtCamera4<Scalar>::VecN params;
                params << intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,
                    intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2],
                    intrinsics.coeffs[3];

                // std::cout << "Camera intrinsics: " << params.transpose() << std::endl;

                basalt::GenericCamera<Scalar> camera;
                basalt::KannalaBrandtCamera4 kannala_brandt(params);
                camera.variant = kannala_brandt;

                calib->intrinsics.push_back(camera);
            }
            else
            {
                std::abort();
            }
        }

        return calib;
    }

} // namespace basalt
