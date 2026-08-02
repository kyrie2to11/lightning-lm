#include <gtest/gtest.h>

#include "common/debug_visualization.h"

namespace lightning {
namespace {

TEST(DebugVisualizationParams, DefaultsDisableAllOnlineOutputs) {
    DebugVisualization::Params params;

    EXPECT_FALSE(params.live_cloud_enabled);
    EXPECT_FALSE(params.live_timeseries_enabled);
}

TEST(DebugVisualizationSampling, UsesIndependentFrameStrides) {
    DebugVisualization::Params params;
    params.live_cloud_enabled = true;
    params.live_cloud_every_n_frames = 10;
    params.live_timeseries_enabled = true;
    params.live_timeseries_every_n_frames = 3;

    EXPECT_TRUE(DebugVisualization::ShouldPublishCloud(params, 0, false));
    EXPECT_FALSE(DebugVisualization::ShouldPublishCloud(params, 9, false));
    EXPECT_TRUE(DebugVisualization::ShouldPublishCloud(params, 10, false));
    EXPECT_TRUE(DebugVisualization::ShouldPublishTimeseries(params, 3, false));
    EXPECT_FALSE(DebugVisualization::ShouldPublishTimeseries(params, 4, false));
}

TEST(DebugVisualizationSampling, ForcedEventsStillRespectMasterSwitch) {
    DebugVisualization::Params disabled;
    EXPECT_FALSE(DebugVisualization::ShouldPublishCloud(disabled, 1, true));
    EXPECT_FALSE(DebugVisualization::ShouldPublishTimeseries(disabled, 1, true));

    DebugVisualization::Params enabled;
    enabled.live_cloud_enabled = true;
    enabled.live_timeseries_enabled = true;
    EXPECT_TRUE(DebugVisualization::ShouldPublishCloud(enabled, 1, true));
    EXPECT_TRUE(DebugVisualization::ShouldPublishTimeseries(enabled, 1, true));
}

TEST(DebugVisualizationParams, RejectsInvalidEnabledStride) {
    DebugVisualization::Params params;
    params.live_cloud_enabled = true;
    params.live_cloud_every_n_frames = 0;

    EXPECT_THROW(DebugVisualization::Validate(params), std::invalid_argument);
}

}  // namespace
}  // namespace lightning
