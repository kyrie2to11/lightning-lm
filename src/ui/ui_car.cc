#include "ui/ui_car.h"
#include <GL/gl.h>

namespace lightning::ui {

std::vector<Vec3f> UiCar::car_vertices_ = {
    // clang-format off
     { 0, 0, 0}, { 3.0, 0, 0},
     { 0, 0, 0}, { 0, 3.0, 0},
     { 0, 0, 0}, { 0, 0, 3.0},
    // clang-format on
};

void UiCar::SetPose(const SE3& pose) {
    pts_.clear();
    for (auto& p : car_vertices_) {
        pts_.emplace_back(p);
    }

    // 转换到世界系
    auto pose_f = pose.cast<float>();
    for (auto& pt : pts_) {
        pt = pose_f * pt;
    }
}

void UiCar::Render() {
    glLineWidth(5.0);
    glBegin(GL_LINES);

    /// X-红, Y-绿, Z-蓝（对照 RViz base_footprint: 红=X前, 绿=Y左, 蓝=Z上）
    glColor3f(1.0f, 0.0f, 0.0f);  // X 红
    glVertex3f(pts_[0][0], pts_[0][1], pts_[0][2]);
    glVertex3f(pts_[1][0], pts_[1][1], pts_[1][2]);

    glColor3f(0.0f, 1.0f, 0.0f);  // Y 绿
    glVertex3f(pts_[2][0], pts_[2][1], pts_[2][2]);
    glVertex3f(pts_[3][0], pts_[3][1], pts_[3][2]);

    glColor3f(0.0f, 0.0f, 1.0f);  // Z 蓝
    glVertex3f(pts_[4][0], pts_[4][1], pts_[4][2]);
    glVertex3f(pts_[5][0], pts_[5][1], pts_[5][2]);
    glEnd();
}

}  // namespace lightning::ui
