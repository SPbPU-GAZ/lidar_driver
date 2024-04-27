load("//tools/install:install.bzl", "install")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "ls180s2_gazel/lib",
    data_dest = "ls180s2_gazel",
    data = ["//modules/drivers:cyberfile.xml"],
    runtime_dest = "ls180s2_gazel/bin",
    targets = [
        "//modules/drivers/lidar/ls180s2_gazel/driver:lslidar_library",
        "//modules/drivers/lidar/ls180s2_gazel/driver:driver",
        "//modules/drivers/lidar/ls180s2_gazel/driver:lslidar_binary.so",
        # "//modules/drivers/lidar/ls180s2_gazel/driver:lslidar_binary.so",
        # "//modules/drivers/lidar/ls180s2_gazel/driver:parser_binary.so",
    ],
    visibility = ["//visibility:public"],
)

cpplint()