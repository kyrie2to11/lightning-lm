#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/localization/localization_visualization.h"
#include "ui/ui_trajectory.h"

namespace lightning::loc {
namespace {

TEST(LocalizationVisualization, TransformsNdtScanIntoMapFrame) {
    PointCloudType scan;
    PointType point;
    point.x = 1.0F;
    point.y = 2.0F;
    point.z = 3.0F;
    point.intensity = 42.0F;
    scan.push_back(point);

    const SE3 map_from_scan(Eigen::Quaterniond::Identity(), Vec3d(10.0, -4.0, 0.5));
    const auto message = MakeAlignedScanMessage(scan, map_from_scan, 123.25);

    EXPECT_EQ(message.header.frame_id, "map");
    EXPECT_EQ(message.header.stamp.sec, 123);
    EXPECT_EQ(message.header.stamp.nanosec, 250000000U);

    PointCloudType aligned;
    pcl::fromROSMsg(message, aligned);
    ASSERT_EQ(aligned.size(), 1U);
    EXPECT_FLOAT_EQ(aligned.front().x, 11.0F);
    EXPECT_FLOAT_EQ(aligned.front().y, -2.0F);
    EXPECT_FLOAT_EQ(aligned.front().z, 3.5F);
    EXPECT_FLOAT_EQ(aligned.front().intensity, 42.0F);
}

TEST(PangolinLocalizationVisualization, UsesPointsForNdtAndLinesForPgo) {
    const ui::UiTrajectory ndt(
        Vec3f(0.0F, 1.0F, 0.0F), ui::TrajectoryPrimitive::POINTS, 4.0F);
    const ui::UiTrajectory pgo(
        Vec3f(1.0F, 1.0F, 0.0F), ui::TrajectoryPrimitive::LINE_STRIP, 3.0F,
        ui::TrajectoryLineStyle::DASHED);
    const ui::UiTrajectory final_pose(
        Vec3f(1.0F, 0.0F, 0.0F), ui::TrajectoryPrimitive::LINE_STRIP, 3.0F);

    EXPECT_EQ(ndt.Primitive(), ui::TrajectoryPrimitive::POINTS);
    EXPECT_FLOAT_EQ(ndt.RenderSize(), 4.0F);
    EXPECT_EQ(pgo.Primitive(), ui::TrajectoryPrimitive::LINE_STRIP);
    EXPECT_FLOAT_EQ(pgo.RenderSize(), 3.0F);
    EXPECT_EQ(pgo.LineStyle(), ui::TrajectoryLineStyle::DASHED);
    EXPECT_EQ(final_pose.Primitive(), ui::TrajectoryPrimitive::LINE_STRIP);
    EXPECT_FLOAT_EQ(final_pose.RenderSize(), 3.0F);
    EXPECT_EQ(final_pose.LineStyle(), ui::TrajectoryLineStyle::SOLID);
}

}  // namespace
}  // namespace lightning::loc
