#pragma once

#include "common/eigen_types.h"

#include <pangolin/gl/glvbo.h>

namespace lightning::ui {

enum class TrajectoryPrimitive { LINE_STRIP, POINTS };
enum class TrajectoryLineStyle { SOLID, DASHED };

/// UI中的轨迹绘制
class UiTrajectory {
   public:
    UiTrajectory(
        const Vec3f& color, TrajectoryPrimitive primitive = TrajectoryPrimitive::LINE_STRIP,
        float render_size = 5.0F, TrajectoryLineStyle line_style = TrajectoryLineStyle::SOLID)
        : primitive_(primitive), line_style_(line_style), render_size_(render_size), color_(color) {
        pos_.reserve(max_size_);
    }

    /// 增加一个轨迹点到opengl缓冲区
    void AddPt(const SE3& pose);

    /// 渲染此轨迹
    void Render();

    void Clear() {
        pos_.clear();
        pos_.reserve(max_size_);
        vbo_.Free();
    }

    Vec3f At(const uint64_t idx) const { return pos_.at(idx); }
    TrajectoryPrimitive Primitive() const { return primitive_; }
    TrajectoryLineStyle LineStyle() const { return line_style_; }
    float RenderSize() const { return render_size_; }

   private:
    int max_size_ = 1e6;                               // 记录的最大点数
    std::vector<Eigen::Vector3f> pos_;                 // 轨迹记录数据
    TrajectoryPrimitive primitive_ = TrajectoryPrimitive::LINE_STRIP;
    TrajectoryLineStyle line_style_ = TrajectoryLineStyle::SOLID;
    float render_size_ = 5.0F;
    Eigen::Vector3f color_ = Eigen::Vector3f::Zero();  // 轨迹颜色显示
    pangolin::GlBuffer vbo_;                           // 显存顶点信息
};

}  // namespace lightning::ui
