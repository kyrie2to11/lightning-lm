#include <algorithm>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include "core/maps/tiled_map.h"

namespace lightning {
namespace {

TEST(TiledMapVisualization, ReturnsAllIndexedStaticChunksBeforeLocalizationStarts) {
    const auto map_dir = std::filesystem::temp_directory_path() /
                         ("lightning_tiled_map_visualization_" +
                          std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::remove_all(map_dir);
    std::filesystem::create_directories(map_dir);

    PointCloudType first;
    PointType first_point;
    first_point.x = 1.0F;
    first_point.y = 2.0F;
    first_point.z = 3.0F;
    first.emplace_back(first_point);
    pcl::io::savePCDFileBinaryCompressed((map_dir / "0.pcd").string(), first);

    PointCloudType second;
    PointType second_point;
    second_point.x = 101.0F;
    second_point.y = 2.0F;
    second_point.z = 3.0F;
    second.emplace_back(second_point);
    pcl::io::savePCDFileBinaryCompressed((map_dir / "1.pcd").string(), second);

    std::ofstream index(map_dir / "index.txt");
    index << "0 0 0\n"
          << "0 0 0 " << (map_dir / "0.pcd").string() << "\n"
          << "1 1 0 " << (map_dir / "1.pcd").string() << "\n";
    index.close();

    TiledMap::Options options;
    options.map_path_ = map_dir.string();
    TiledMap map(options);
    ASSERT_TRUE(map.LoadMapIndex());

    const CloudPtr visualization_map = map.GetFullStaticMap();

    ASSERT_NE(visualization_map, nullptr);
    EXPECT_EQ(visualization_map->size(), 2U);
    std::vector<float> x_coordinates;
    for (const auto& point : *visualization_map) {
        x_coordinates.emplace_back(point.x);
    }
    std::sort(x_coordinates.begin(), x_coordinates.end());
    EXPECT_EQ(x_coordinates, (std::vector<float>{1.0F, 101.0F}));

    std::filesystem::remove_all(map_dir);
}

}  // namespace
}  // namespace lightning
