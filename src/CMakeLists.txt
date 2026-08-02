add_library(${PROJECT_NAME}.libs SHARED
        common/nav_state.cc
        common/constant.h
        common/data_capture.cc
        common/debug_visualization.cc
        common/options.cc
        common/params.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/eskf.cc
        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
        core/loop_closing/loop_closing.cc

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc

        core/localization/localization_result.cc
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
        core/localization/localization.cpp

        utils/timer.cc
        utils/pointcloud_utils.cc

        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_cloud.cc
        ui/ui_trajectory.cc

        core/system/slam.cc
        core/system/loc_system.cc

        wrapper/bag_io.cc
)

ament_target_dependencies(
        ${PROJECT_NAME}.libs
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "plotjuggler_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
        "tf2_eigen"
)

add_dependencies(${PROJECT_NAME}.libs
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

target_link_libraries(${PROJECT_NAME}.libs
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}.libs miao.core miao.utils
        DESTINATION lib)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_pointcloud_preprocess
            ${PROJECT_SOURCE_DIR}/test/test_pointcloud_preprocess.cc)
    target_link_libraries(test_pointcloud_preprocess
            ${PROJECT_NAME}.libs
            ${third_party_libs})
    target_compile_definitions(test_pointcloud_preprocess PRIVATE
            LIGHTNING_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

    ament_add_gtest(test_imu_initialization
            ${PROJECT_SOURCE_DIR}/test/test_imu_initialization.cc)
    target_link_libraries(test_imu_initialization
            ${PROJECT_NAME}.libs
            ${third_party_libs})

    ament_add_gtest(test_data_capture
            ${PROJECT_SOURCE_DIR}/test/test_data_capture.cc)
    target_link_libraries(test_data_capture
            ${PROJECT_NAME}.libs
            ${third_party_libs}
            ${PCL_LIBRARIES})

    ament_add_gtest(test_eskf_iteration_capture
            ${PROJECT_SOURCE_DIR}/test/test_eskf_iteration_capture.cc)
    target_link_libraries(test_eskf_iteration_capture
            ${PROJECT_NAME}.libs
            ${third_party_libs})

    ament_add_gtest(test_ivox_snapshot
            ${PROJECT_SOURCE_DIR}/test/test_ivox_snapshot.cc)
    target_link_libraries(test_ivox_snapshot
            ${PROJECT_NAME}.libs
            ${third_party_libs})

    ament_add_gtest(test_debug_visualization
            ${PROJECT_SOURCE_DIR}/test/test_debug_visualization.cc)
    target_link_libraries(test_debug_visualization
            ${PROJECT_NAME}.libs
            ${third_party_libs})
endif()

add_subdirectory(app)
