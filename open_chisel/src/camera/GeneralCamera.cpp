#include <open_chisel/camera/GeneralCamera.h>

namespace chisel
{

    GeneralCamera::GeneralCamera()
    {


    }

    GeneralCamera::~GeneralCamera()
    {

    }

    Vec3 GeneralCamera::ProjectPoint(const Vec3& point) const
    {
        if(cameratype == CameraType::Pinhole)
        {
            const float& x = point(0);
            const float& y = point(1);
            const float& z = point(2);
            const float invZ = 1.0f / z;
            return Vec3(intrinsics.GetFx() * x * invZ + intrinsics.GetCx(), intrinsics.GetFy() * y * invZ + intrinsics.GetCy(), z);
        }
        else
        {
            const float theta = acos(point(2) / point.norm()) * 57.29578; //57.29577 = 180 / pi
            const float phi = atan2(point(1), point(0));
            return Vec3(theta * degree_step * cos(phi) + width / 2, theta * degree_step * sin(phi) + height / 2, point(2));
        }
    }

    Vec3 GeneralCamera::UnprojectPoint(const Vec3& point) const
    {
        if(cameratype == CameraType::Pinhole)
        {
            const float& u = point(0);
            const float& v = point(1);
            const float& z = point(2);
            return Vec3(z * ((u - intrinsics.GetCx()) / intrinsics.GetFx()), z * ((v - intrinsics.GetCy()) / intrinsics.GetFy()), z);
        }
        else
        {
            Vec2 point_cam = Vec2(point(0) - width / 2, point(1) - height / 2);
            const float theta = point_cam.norm() / degree_step * 0.0174532; // 0.0174532 = pi / 180;
            const float phi = atan2(point_cam(1), point_cam(0));
            return Vec3(cos(phi) * sin(theta), sin(phi) * sin(theta), cos(theta));
        }
    }

    void GeneralCamera::SetupFrustum(const Transform& view, Frustum* frustum) const
    {
        assert(frustum != nullptr);
        if(cameratype == CameraType::Pinhole)
        {
            frustum->SetFromParams(
                view, nearPlane, farPlane, intrinsics.GetFx(), intrinsics.GetFy(), intrinsics.GetCx(), intrinsics.GetCy(), width, height);
        }
        else
        {
            frustum->SetFromParams(
                view, farPlane);
        }
    }

    bool GeneralCamera::IsPointOnImage(const Vec3& point) const
    {
        return point(0) >= 0 && point(1) >= 0 && point(0) < width && point(1) < height;        
    }

} // namespace chisel 
