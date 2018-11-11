// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "backward.hpp"
#define BACKWARD_HAS_BFD 1
namespace backward {
backward::SignalHandling sh;
}

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <chisel_ros/ChiselServer.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/truncation/InverseTruncator.h>
#include <open_chisel/truncation/ConstantTruncator.h>

int main(int argc, char **argv)
{
    ROS_INFO("Starting up chisel node.");
    ros::init(argc, argv, "Chisel");
    ros::NodeHandle nh("~");
    int chunkSizeX, chunkSizeY, chunkSizeZ;
    double voxelResolution;
    double truncationDistScale;
    int weight;
    bool useCarving;
    bool useColor;
    double carvingDist;
    bool pointcloud_transformed(false);//true: the pointcloud is transformed to the world frame;

    std::string depthImageTopic;
    std::string depthImageInfoTopic;
    std::string depthImageTransform;//useless
    std::string colorImageTopic;
    std::string colorImageInfoTopic;
    std::string colorImageTransform;//useless
    std::string baseTransform;
    std::string meshTopic;
    std::string chunkBoxTopic;
    double nearPlaneDist;
    double farPlaneDist;
    chisel_ros::ChiselServer::FusionMode mode;
    std::string modeString;
    std::string cameraString;
    std::string depthTopic_name;
    std::string colorTopic_name;
    std::string odometryTopic_name;

    nh.param("chunk_size_x", chunkSizeX, 32);
    nh.param("chunk_size_y", chunkSizeY, 32);
    nh.param("chunk_size_z", chunkSizeZ, 32);
    nh.param("truncation_scale", truncationDistScale, 8.0);
    nh.param("integration_weight", weight, 1);
    nh.param("use_voxel_carving", useCarving, true);
    nh.param("use_color", useColor, true);
    nh.param("carving_dist_m", carvingDist, 0.05);
    nh.param("voxel_resolution_m", voxelResolution, 0.03);
    nh.param("near_plane_dist", nearPlaneDist, 0.05);
    nh.param("far_plane_dist", farPlaneDist, 5.0);

    // nh.param("depth_image_topic", depthImageTopic, std::string("/depth_image"));
    // nh.param("point_cloud_topic", pointCloudTopic, std::string("/pointcloud"));
    // nh.param("depth_image_info_topic", depthImageInfoTopic, std::string("/depth_camera_info"));
    // nh.param("depth_image_transform", depthImageTransform, std::string("/camera_depth_optical_frame"));
    // nh.param("color_image_topic", colorImageTopic, std::string("/color_image"));
    // nh.param("color_image_info_topic", colorImageInfoTopic, std::string("/color_camera_info"));
    // nh.param("color_image_transform", colorImageTransform, std::string("/camera_rgb_optical_frame"));
    // nh.param("base_transform", baseTransform, std::string("/camera_link"));
    nh.param("mesh_topic", meshTopic, std::string("full_mesh"));
    nh.param("chunk_box_topic", chunkBoxTopic, std::string("chunk_boxes"));
    nh.param("fusion_mode", modeString, std::string("DepthImage"));
    nh.param("camera_type", cameraString, std::string("Pinhole"));
    nh.param("Odometry_topic", odometryTopic_name, std::string("/Odometry"));
    nh.param("Depthimage_topic", depthTopic_name, std::string("/DepthImage"));
    nh.param("Colorimage_topic", colorTopic_name, std::string("/ColorImage"));
    // nh.param("Pointlcoud_is_in_worldframe", pointcloud_transformed, false);


    if (modeString == "DepthImage")
    {
        ROS_INFO("Mode depth image");
        mode = chisel_ros::ChiselServer::FusionMode::DepthImage;
    }
    else if (modeString == "PointCloud")
    {
        ROS_INFO("Mode point cloud");
        mode = chisel_ros::ChiselServer::FusionMode::PointCloud;
    }
    else
    {
        ROS_ERROR("Unrecognized fusion mode %s. Recognized modes: \"DepthImage\", \"PointCloud\"\n", modeString.c_str());
        return -1;
    }

    ROS_INFO("Subscribing.");

    chisel_ros::ChiselServerPtr server(new chisel_ros::ChiselServer(nh, chunkSizeX, chunkSizeY, chunkSizeZ, voxelResolution, useColor, mode));

    chisel::TruncatorPtr truncator(new chisel::ConstantTruncator(truncationDistScale));
    //chisel::TruncatorPtr truncator(new chisel::QuadraticTruncator(truncationDistScale));
    // chisel::TruncatorPtr truncator(new chisel::InverseTruncator(truncationDistScale));


    server->SetupProjectionIntegrator(truncator, static_cast<uint16_t>(weight), useCarving, carvingDist);

    //ROS_ASSERT(mode == chisel_ros::ChiselServer::FusionMode::DepthImage);
    //ROS_ASSERT(useColor && mode == chisel_ros::ChiselServer::FusionMode::DepthImage);

    server->SetNearPlaneDist(nearPlaneDist);
    server->SetFarPlaneDist(farPlaneDist);

    if (mode == chisel_ros::ChiselServer::FusionMode::PointCloud)
    {
        // server->Subscribe_pointcloud_All(pointCloudTopic, odometryTopic_name, pointcloud_transformed);
    }

    if (mode == chisel_ros::ChiselServer::FusionMode::DepthImage)
    {
        server->Subscribe_image_All(  depthTopic_name,
                                      colorTopic_name,
                                      odometryTopic_name);
    }

    if( cameraString == "Pinhole")
    {
      float fx, fy, cx, cy;
      int image_width;
      int image_height;
      bool has_all_param = true;
      has_all_param &= nh.getParam("fx", fx);
      has_all_param &= nh.getParam("fy", fy);
      has_all_param &= nh.getParam("cx", cx);
      has_all_param &= nh.getParam("cy", cy);
      has_all_param &= nh.getParam("image_width", image_width);
      has_all_param &= nh.getParam("image_height", image_height);
      if(has_all_param)
        server->setPinholeCameraType(fx, fy, cx, cy, image_width, image_height, farPlaneDist, nearPlaneDist);
      else
        ROS_ERROR("no enough param for Pinhole camera model");
      ROS_INFO("Mode Pinhole Camera.");
    }
    else
    {
      int image_width;
      int image_height;
      int degree_step;
      bool has_all_param = true;
      has_all_param &= nh.getParam("image_width", image_width);
      has_all_param &= nh.getParam("image_height", image_height);
      has_all_param &= nh.getParam("degree_step", degree_step);
      if(has_all_param)
        server->setFisheyeCameraType(degree_step, image_width, image_height, farPlaneDist);
      else
        ROS_ERROR("no enough param for Fisheye camera model");
      ROS_INFO("Mode Fisheye Camera.");
    }

    // server->SetupDepthPosePublisher("last_depth_pose");
    // server->SetupDepthFrustumPublisher("last_depth_frustum");

    // server->SetupColorPosePublisher("last_color_pose");
    // server->SetupColorFrustumPublisher("last_color_frustum");

    server->AdvertiseServices();

    server->SetBaseTransform(baseTransform);
    server->SetupMeshPublisher(meshTopic);
    server->SetupChunkBoxPublisher(chunkBoxTopic);
    ROS_INFO("Beginning to loop.");

    ros::spin();
}
