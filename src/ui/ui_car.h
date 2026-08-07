#pragma once

#include "common/eigen_types.h"

namespace lightning::ui {

/// 在UI里显示的小车
class UiCar {
   public:
    UiCar(const Vec3f& color, float axis_length = 3.0f, float line_width = 5.0f, bool dashed = false)
        : color_(color), axis_length_(axis_length), line_width_(line_width), dashed_(dashed) {}

    /// 设置小车 Pose，重设显存中的点
    void SetPose(const SE3& pose);

    /// 渲染小车
    void Render();

   private:
    Vec3f color_;
    float axis_length_ = 3.0f;
    float line_width_ = 5.0f;
    bool dashed_ = false;  // 后端位姿轴用虚线，与前端实线区分
    std::vector<Vec3f> pts_;
};

}  // namespace lightning::ui
