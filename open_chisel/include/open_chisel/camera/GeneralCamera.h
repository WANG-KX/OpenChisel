#ifndef GENERALCAMERA_H_
#define GENERALCAMERA_H_

#include <memory>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/camera/Intrinsics.h>

namespace chisel
{

    class GeneralCamera
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            enum class CameraType
            {
                Pinhole,
                Fisheye
            };

            GeneralCamera();
            virtual ~GeneralCamera();

            inline const Intrinsics& GetIntrinsics() const { return intrinsics; }
            inline Intrinsics& GetMutableIntrinsics() { return intrinsics; }
            inline void SetIntrinsics(const Intrinsics& value) { intrinsics = value; }

            inline int GetWidth() const { return width; }
            inline int GetHeight() const { return height; }
            inline void SetType(CameraType camtype) { cameratype = camtype; }
            inline void SetWidth(int value) { width = value; }
            inline void SetHeight(int value) { height = value; }
            inline void SetIntrinsics(float fx, float fy, float cx, float cy)
                {intrinsics.SetFx(fx);intrinsics.SetFy(fy);intrinsics.SetCx(cx);intrinsics.SetCy(cy);}
            inline void SetDegreeStep(int step) { degree_step = (float)step; }
            inline float GetNearPlane() const { return nearPlane; }
            inline float GetFarPlane() const { return farPlane; }
            inline void SetNearPlane(float value) { nearPlane = value; }
            inline void SetFarPlane(float value) { farPlane = value; }

            void SetupFrustum(const Transform& view, Frustum* frustum) const;

            Vec3 ProjectPoint(const Vec3& point) const;
            Vec3 UnprojectPoint(const Vec3& point) const;

            bool IsPointOnImage(const Vec3& point) const;

        protected:
            Intrinsics intrinsics;
            CameraType cameratype;
            int width;
            int height;
            float degree_step;
            float nearPlane;
            float farPlane;
    };
    typedef std::shared_ptr<GeneralCamera> GeneralCameraPtr;
    typedef std::shared_ptr<const GeneralCamera> GeneralCameraConstPtr;
} // namespace chisel 

#endif // GENERALCAMERA_H_ 
