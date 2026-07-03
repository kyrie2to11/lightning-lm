//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "", "地图保存根目录；非空时覆盖配置文件中的 system.map_path");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    using namespace lightning;

    auto non_ros_args = NonRosArguments(argc, argv);
    std::vector<char*> gflags_argv;
    gflags_argv.reserve(non_ros_args.size());
    for (auto& arg : non_ros_args) {
        gflags_argv.push_back(arg.data());
    }
    int gflags_argc = static_cast<int>(gflags_argv.size());
    char** gflags_argv_data = gflags_argv.data();
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_data, true);

    /// 需要rclcpp::init
    rclcpp::init(argc, argv);

    SlamSystem::Options options;
    options.online_mode_ = true;
    options.map_path_ = FLAGS_map_path;

    SlamSystem slam(options);
    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");
    slam.Spin();

    Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}
