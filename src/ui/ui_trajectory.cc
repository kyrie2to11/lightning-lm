#include "ui/ui_trajectory.h"

#include <GL/gl.h>

namespace lightning::ui {

void UiTrajectory::AddPt(const SE3& pose) {
    // 如果轨迹点超出阈值 直接删除前一半点
    pos_.emplace_back(pose.translation().cast<float>());
    if (pos_.size() > max_size_) {
        pos_.erase(pos_.begin(), pos_.begin() + pos_.size() / 2);
    }
}

void UiTrajectory::Render() {
    const bool render_points = primitive_ == TrajectoryPrimitive::POINTS;
    const bool render_dashed = !render_points && line_style_ == TrajectoryLineStyle::DASHED;
    if (render_points) {
        glPointSize(render_size_);
    } else {
        glLineWidth(render_size_);
    }
    if (render_dashed) {
        glEnable(GL_LINE_STIPPLE);
        glLineStipple(2, 0x00FF);
    }
    glBegin(render_points ? GL_POINTS : GL_LINE_STRIP);
    glColor3f(color_[0], color_[1], color_[2]);

    for (const auto& p : pos_) {
        glVertex3f(p[0], p[1], p[2]);
    }
    glEnd();

    glLineWidth(1.0F);
    glPointSize(1.0F);
    if (render_dashed) {
        glDisable(GL_LINE_STIPPLE);
    }
}

}  // namespace lightning::ui
