load("//tools/install:install.bzl", "install")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LSLIDAR_COPTS = ['-DMODULE_NAME=\\"ls180s2_gazel\\"']


cc_library(
    name = "parser_base_library",
    srcs = [
        "parser.cc", 
        ],
    hdrs = [
        "parser.h", 
        ],
    copts = LSLIDAR_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:lslidar_proto_cc",
        "//modules/drivers/lidar/proto:lslidar_proto_config_cc",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/ls180s2_gazel/driver:lslidar_library",
        "@local_config_pcl//:pcl",
        "@eigen",
        ],
    linkopts = [
        "-lpthread",
        "-lpcap",
        ],
)

cc_library(
    name = "parser_factory_library",
    srcs = [
        "parser_factory.cc", 
        ],
    hdrs = [
        "parser_factory.h"
        ],
    copts = LSLIDAR_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:lslidar_proto_cc",
        "//modules/drivers/lidar/proto:lslidar_proto_config_cc",
        ":parser_base_library",
        "@eigen",
        ],
    linkopts = [
        "-lpthread",
        "-lpcap",
        ],
)

# cc_binary(
#     name = "parser_binary.so",
#     linkshared = True,
#     linkstatic = True,
#     deps = [
#         "//cyber",
#         ":parser_base_library",
#         ":parser_factory_library",
#     ],
# )


cpplint()