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

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/DepthImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace chisel_ros
{
void PclPointCloudToChisel(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, chisel::PointCloud *cloudOut, std::vector<float> &certainity)
{
    assert(!!cloudOut);
    cloudOut->GetMutablePoints().resize(cloudIn.points.size());
    certainity.clear();
    certainity.reserve(cloudIn.points.size());
    size_t i = 0;
    for (const pcl::PointXYZI &pt : cloudIn.points)
    {
        chisel::Vec3 &xyz = cloudOut->GetMutablePoints().at(i);
        xyz(0) = pt.x;
        xyz(1) = pt.y;
        xyz(2) = pt.z;
        certainity.push_back(pt.intensity);
        i++;
    }
}

void PclColorPointCloudToChisel(const pcl::PointCloud<pcl::PointXYZRGB> &cloudIn, chisel::PointCloud *cloudOut)
{
    assert(!!cloudOut);
    cloudOut->GetMutablePoints().resize(cloudIn.points.size());
    cloudOut->GetMutableColors().resize(cloudIn.points.size());
    size_t i = 0;
    float byteToFloat = 1.0f / 255.0f;
    for (const pcl::PointXYZRGB &pt : cloudIn.points)
    {
        chisel::Vec3 &xyz = cloudOut->GetMutablePoints().at(i);
        xyz(0) = pt.x;
        xyz(1) = pt.y;
        xyz(2) = pt.z;

        chisel::Vec3 &rgb = cloudOut->GetMutableColors().at(i);
        rgb(0) = pt.r * byteToFloat;
        rgb(1) = pt.g * byteToFloat;
        rgb(2) = pt.b * byteToFloat;
        i++;
    }
}

void ROSPointCloudToChisel(sensor_msgs::PointCloud2ConstPtr cloudIn, chisel::PointCloud *cloudOut)
{
    // assert(!!cloudOut);

    // if (sensor_msgs::getPointCloud2FieldIndex(*cloudIn, "rgb") == -1)
    // {
    //     //Got uncolored pointcloud
    //     pcl::PointCloud<pcl::PointXYZI> pclCloud;
    //     pcl::fromROSMsg(*cloudIn, pclCloud);
    //     //remove NAN points from the cloud
    //     std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(pclCloud, pclCloud, indices);
    //     PclPointCloudToChisel(pclCloud, cloudOut);
    //     printf("format changed size : ", pclCloud.size());
    // }
    // else
    // {
    //     //Got colored pointcloud
    //     pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
    //     pcl::fromROSMsg(*cloudIn, pclCloud);
    //     //remove NAN points from the cloud
    //     std::vector<int> indices;
    //     pcl::removeNaNFromPointCloud(pclCloud, pclCloud, indices);
    //     PclColorPointCloudToChisel(pclCloud, cloudOut);
    // }
}

//??? wang: this is used to transform the pointcloud back into the camera/ladar frame
void ROSPointCloudToChisel_with_inverse_trans(
    sensor_msgs::PointCloud2ConstPtr cloudIn,
    chisel::PointCloud *cloudOut,
    chisel::Transform transform,
    std::vector<float> &certainity)
{
    assert(!!cloudOut);

    if (sensor_msgs::getPointCloud2FieldIndex(*cloudIn, "rgb") == -1)
    {
        //Got uncolored pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud_out(new  pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloudIn, *pclCloud);

        //filter 
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud (pclCloud);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.setMinimumPointsNumberPerVoxel(10);
        sor.filter (*pclCloud_out);

        //transform back into camera/ladar frame
        pcl::PointCloud<pcl::PointXYZI> pclCloud_transformed;
        pcl::transformPointCloud(*pclCloud_out, pclCloud_transformed, transform.inverse());

        //remove NAN points from the cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(pclCloud_transformed, pclCloud_transformed, indices);
        PclPointCloudToChisel(pclCloud_transformed, cloudOut, certainity);
        printf("format changed size : %d.\n", pclCloud_out->size());
    }
    else
    {
        //Got colored pointcloud
        pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
        pcl::fromROSMsg(*cloudIn, pclCloud);

        //transform back into camera/ladar frame
        pcl::PointCloud<pcl::PointXYZRGB> pclCloud_transformed;
        pcl::transformPointCloud(pclCloud, pclCloud_transformed, transform.inverse());

        //remove NAN points from the cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(pclCloud_transformed, pclCloud_transformed, indices);
        PclColorPointCloudToChisel(pclCloud_transformed, cloudOut);
    }
}


template <class DataType>
void ROSImgToDepthImg(sensor_msgs::ImageConstPtr image, chisel::DepthImage<DataType> *depthImage)
{
    ROS_INFO("Got depth image of format %s", image->encoding.c_str());
    bool mmImage = false;

    if (image->encoding == "16UC1")
    {
        mmImage = true;
    }
    else if (image->encoding == "32FC1")
    {
        mmImage = false;
    }
    else
    {
        ROS_ERROR("Unrecognized depth image format.");
        return;
    }

    if (!mmImage)
    {
        size_t dataSize = image->step / image->width;
        assert(depthImage->GetHeight() == static_cast<int>(image->height) && depthImage->GetWidth() == static_cast<int>(image->width));
        assert(dataSize == sizeof(DataType));
        const DataType *imageData = reinterpret_cast<const DataType *>(image->data.data());
        DataType *depthImageData = depthImage->GetMutableData();
        int totalPixels = image->width * image->height;
        for (int i = 0; i < totalPixels; i++)
        {
            depthImageData[i] = imageData[i];
        }
    }
    else
    {
        assert(depthImage->GetHeight() == static_cast<int>(image->height) && depthImage->GetWidth() == static_cast<int>(image->width));
        const uint16_t *imageData = reinterpret_cast<const uint16_t *>(image->data.data());
        DataType *depthImageData = depthImage->GetMutableData();
        int totalPixels = image->width * image->height;
        for (int i = 0; i < totalPixels; i++)
        {
            depthImageData[i] = (1.0f / 1000.0f) * imageData[i];
        }
    }
}

template <class DataType>
void ROSImgToColorImg(sensor_msgs::ImageConstPtr image, chisel::ColorImage<DataType> *colorImage)
{
    size_t numChannels = colorImage->GetNumChannels();
    size_t dataSize = image->step / image->width;
    assert(colorImage->GetHeight() == static_cast<int>(image->height) && colorImage->GetWidth() == static_cast<int>(image->width));

    if (dataSize != numChannels * sizeof(DataType))
    {
        ROS_ERROR("Inconsistent channel width: %lu. Expected %lu\n", dataSize, numChannels * sizeof(DataType));
        return;
    }

    const DataType *imageData = reinterpret_cast<const DataType *>(image->data.data());
    DataType *colorImageData = colorImage->GetMutableData();
    int totalPixels = image->width * image->height * numChannels;
    for (int i = 0; i < totalPixels; i++)
    {
        colorImageData[i] = imageData[i];
    }
}

template <class DataType>
chisel::ColorImage<DataType> *ROSImgToColorImg(sensor_msgs::ImageConstPtr image)
{
    size_t numChannels = 0;

    if (image->encoding == "mono8")
    {
        numChannels = 1;
    }
    else if (image->encoding == "bgr8" || image->encoding == "rgb8")
    {
        numChannels = 3;
    }
    else if (image->encoding == "bgra8")
    {
        numChannels = 4;
    }
    else
    {
        ROS_ERROR("Unsupported color image format %s. Supported formats are mono8, rgb8, bgr8, and bgra8\n", image->encoding.c_str());
    }

    chisel::ColorImage<DataType> *toReturn = new chisel::ColorImage<DataType>(image->width, image->height, numChannels);
    ROSImgToColorImg(image, toReturn);
    return toReturn;
}

inline chisel::Transform RosTfToChiselTf(const tf::StampedTransform &tf)
{
    chisel::Transform transform;
    transform.translation()(0) = tf.getOrigin().x();
    transform.translation()(1) = tf.getOrigin().y();
    transform.translation()(2) = tf.getOrigin().z();

    chisel::Quaternion quat;
    quat.x() = tf.getRotation().x();
    quat.y() = tf.getRotation().y();
    quat.z() = tf.getRotation().z();
    quat.w() = tf.getRotation().w();
    transform.linear() = quat.toRotationMatrix();

    return transform.inverse();
}

inline chisel::PinholeCamera RosCameraToChiselCamera(const sensor_msgs::CameraInfoConstPtr &camera)
{
    chisel::PinholeCamera cameraToReturn;
    chisel::Intrinsics intrinsics;
    intrinsics.SetFx(camera->P[0]);
    intrinsics.SetFy(camera->P[5]);
    intrinsics.SetCx(camera->P[2]);
    intrinsics.SetCy(camera->P[6]);
    cameraToReturn.SetIntrinsics(intrinsics);
    cameraToReturn.SetWidth(camera->width);
    cameraToReturn.SetHeight(camera->height);
    return cameraToReturn;
}
}

#endif // CONVERSIONS_H_
