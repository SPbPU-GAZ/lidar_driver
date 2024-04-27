load("//tools/install:install.bzl", "install")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LSLIDAR_COPTS = ['-DMODULE_NAME=\\"ls180s2_gazel\\"']

# install(
#     name = "install",
#     library_dest = "ls180s2_gazel/lib",
#     data_dest = "ls180s2_gazel",
#     data = ["//modules/drivers:cyberfile.xml"],
#     runtime_dest = "ls180s2_gazel/bin",
#     targets = [
#         ":lslidar_library",
#         ":lslidar_binary",
#     ],
#     visibility = ["//visibility:public"],
# )

cc_library(
    name = "pcap_library",
    hdrs = [
        "pcap.h"
        ],
    copts = LSLIDAR_COPTS,
    linkopts = [
        "-lpthread",
        "-lpcap",
    ],
)

cc_library(
    name = "lslidar_library",
    srcs = [
        "input.cc", 
        "driver.cc", 
        ],
    hdrs = [
        "input.h", 
        "driver.h", 
        "ThreadPool.h"
        ],
    copts = LSLIDAR_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:lslidar_proto_cc",
        "//modules/drivers/lidar/proto:lslidar_proto_config_cc",
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
        "//modules/drivers/lidar/proto:hesai_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_cc_proto",
        "//modules/drivers/lidar/proto:lidar_parameter_cc_proto",
        "//modules/drivers/lidar/proto:robosense_config_cc_proto",
        ":pcap_library",
        "@local_config_pcl//:pcl",
        "@eigen",
        ],
    linkopts = [
        "-lpthread",
        "-lpcap",
        ],
)

cc_library(
    name = "driver",
    srcs = [
        "lslidar_component.cc", 
        ],
    hdrs = [
        "lslidar_component.h"
        ],
    copts = LSLIDAR_COPTS,
    deps = [
        "//cyber",
        ":lslidar_library",
        "//modules/drivers/lidar/proto:lslidar_proto_cc",
        "//modules/drivers/lidar/proto:lslidar_proto_config_cc",
        "//modules/drivers/lidar/common/driver_factory:driver_base",

        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:hesai_config_cc_proto",
        "//modules/drivers/lidar/proto:hesai_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_cc_proto",
        "//modules/drivers/lidar/proto:lidar_parameter_cc_proto",
        "//modules/drivers/lidar/proto:robosense_config_cc_proto",
        ":pcap_library",
        "@local_config_pcl//:pcl",
        "@eigen",
        ],
    linkopts = [
        "-lpthread",
        "-lpcap",
        ],
)

cc_binary(
    name = "lslidar_binary.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber",
        ":driver",
    ],
)


cpplint()