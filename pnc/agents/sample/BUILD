package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sample_agent",
    srcs = ["PID.cc"],
    hdrs = [
        "PID.h",
        "sample_agent.h",
    ],
    deps = [
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "@glog",
    ],
)
