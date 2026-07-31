#include <gtest/gtest.h>

#include <vector>

#include "core/lio/eskf.hpp"

namespace lightning {
namespace {

TEST(EskfIterationCapture, LabelsObservationCallsAndReportsExecutedUpdates) {
    ESKF filter;
    ESKF::Options options;
    options.max_iterations_ = 2;
    options.epsi_ = ESKF::StateVecType::Constant(1.0);

    std::vector<int> observation_iterations;
    options.lidar_obs_func_ =
        [&observation_iterations](NavState&, ESKF::CustomObservationModel& observation) {
            observation_iterations.push_back(observation.iteration_);
            observation.valid_ = true;
            observation.HTH_.setIdentity();
            observation.HTr_.setZero();
            observation.lidar_residual_mean_ = 0.25;
            observation.lidar_residual_max_ = 0.5;
            observation.effective_feature_count_ = 42;
        };

    std::vector<ESKF::IterationInfo> reports;
    options.iteration_callback_ =
        [&reports](const ESKF::IterationInfo& info) { reports.push_back(info); };
    filter.Init(options);

    filter.Update(ESKF::ObsType::LIDAR, 1.0);

    ASSERT_FALSE(observation_iterations.empty());
    ASSERT_EQ(observation_iterations.size(), reports.size());
    for (std::size_t i = 0; i < reports.size(); ++i) {
        EXPECT_EQ(observation_iterations[i], static_cast<int>(i));
        EXPECT_EQ(reports[i].iteration, static_cast<int>(i));
        EXPECT_EQ(reports[i].effective_feature_count, 42);
        EXPECT_DOUBLE_EQ(reports[i].residual_mean, 0.25);
        EXPECT_DOUBLE_EQ(reports[i].residual_max, 0.5);
        EXPECT_EQ(reports[i].observable_rank, 6);
    }
}

}  // namespace
}  // namespace lightning
