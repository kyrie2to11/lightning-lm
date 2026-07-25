#include "ui/ui_car.h"
#include <GL/gl.h>

namespace lightning::ui {

void UiCar::SetPose(const SE3& pose) {
    pts_.clear();
    float L = axis_length_;
    // X axis vertices, Y axis vertices, Z axis vertices
    std::vector<Vec3f> local = {
        {0, 0, 0}, {L, 0, 0},
        {0, 0, 0}, {0, L, 0},
        {0, 0, 0}, {0, 0, L},
    };

    auto pose_f = pose.cast<float>();
    for (auto& p : local) {
        pts_.emplace_back(pose_f * p);
    }
}

void UiCar::Render() {
    glLineWidth(line_width_);
    glBegin(GL_LINES);

    /// X-红, Y-绿, Z-蓝
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(pts_[0][0], pts_[0][1], pts_[0][2]);
    glVertex3f(pts_[1][0], pts_[1][1], pts_[1][2]);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(pts_[2][0], pts_[2][1], pts_[2][2]);
    glVertex3f(pts_[3][0], pts_[3][1], pts_[3][2]);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(pts_[4][0], pts_[4][1], pts_[4][2]);
    glVertex3f(pts_[5][0], pts_[5][1], pts_[5][2]);
    glEnd();

    // 在每个轴端点画标记点
    glPointSize(8.0f);
    glColor3f(1.0f, 0.5f, 0.5f);
    glBegin(GL_POINTS);
    glVertex3f(pts_[1][0], pts_[1][1], pts_[1][2]);
    glEnd();

    glColor3f(0.5f, 1.0f, 0.5f);
    glBegin(GL_POINTS);
    glVertex3f(pts_[3][0], pts_[3][1], pts_[3][2]);
    glEnd();

    glColor3f(0.5f, 0.5f, 1.0f);
    glBegin(GL_POINTS);
    glVertex3f(pts_[5][0], pts_[5][1], pts_[5][2]);
    glEnd();

    // 恢复默认 state，避免影响后续渲染
    glPointSize(1.0f);
    glLineWidth(1.0f);
}

}  // namespace lightning::ui
