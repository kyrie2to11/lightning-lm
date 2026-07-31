#include <gtest/gtest.h>

#include "common/point_def.h"
#include "core/ivox3d/ivox3d.h"

namespace lightning {
namespace {

TEST(IVoxSnapshot, ReturnsAllStoredMapPoints) {
    using TestIVox = IVox<3, IVoxNodeType::DEFAULT, PointType>;
    TestIVox::Options options;
    options.resolution_ = 1.0F;
    TestIVox ivox(options);

    TestIVox::PointVector input;
    PointType first;
    first.x = 0.0F;
    first.y = 0.0F;
    first.z = 0.0F;
    input.push_back(first);
    PointType second;
    second.x = 2.0F;
    second.y = 0.0F;
    second.z = 0.0F;
    input.push_back(second);
    ivox.AddPoints(input);

    const auto snapshot = ivox.GetAllPoints();

    EXPECT_EQ(snapshot.size(), ivox.NumPoints());
    EXPECT_EQ(snapshot.size(), 2U);
}

}  // namespace
}  // namespace lightning
