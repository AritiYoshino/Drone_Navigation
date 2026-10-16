#!/usr/bin/env python3
"""Replay build compatibility fixes on disposable runtime workspace copies."""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
WORKSPACES = ROOT / "runtime/workspaces"


def rewrite(path, transform):
    before = path.read_text()
    after = transform(before)
    if before != after:
        path.write_text(after)
        print("Prepared", path.relative_to(ROOT))


def relocate_cv_bridge():
    sysroot = ROOT / "runtime/sysroot"
    path = sysroot / "opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake"
    if not path.is_file():
        raise SystemExit("Run scripts/install_opencv.sh before building.")
    def relocate(text):
        for old in ("/usr/lib/aarch64-linux-gnu", "/usr/include", "/opt/ros/melodic/lib"):
            # Normalize an earlier relocation before applying it, so replaying
            # preparation never prefixes an absolute path twice.
            text = text.replace(str(sysroot) + old, old).replace(old, str(sysroot) + old)
        return text
    rewrite(path, relocate)


def control():
    source = WORKSPACES / "Fast-Drone-250-master/src"
    # Melodic requires test registration to be guarded when testing is disabled.
    rewrite(source / "utils/uav_utils/CMakeLists.txt", lambda text: text.replace(
        'catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)\n'
        'if(TARGET ${PROJECT_NAME}-test)\n'
        '  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})\n'
        'endif()',
        'if(CATKIN_ENABLE_TESTING)\n'
        '  catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)\n'
        '  if(TARGET ${PROJECT_NAME}-test)\n'
        '    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})\n'
        '  endif()\n'
        'endif()'))
    rewrite(source / "realflight_modules/px4ctrl/CMakeLists.txt", lambda text: text.replace(
        'add_dependencies(px4ctrl_node quadrotor_msgs)',
        'add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})'))
    # These targets include generated workspace message headers. Explicit
    # dependencies also make a completely fresh parallel build deterministic.
    for relative, targets in [
        ("planner/plan_manage/CMakeLists.txt", ["ego_planner_node", "traj_server"]),
        ("planner/rosmsg_tcp_bridge/CMakeLists.txt", ["bridge_node"]),
    ]:
        def add_dependencies(text, targets=targets):
            marker = "# Runtime migration: generated ROS message prerequisites."
            if marker not in text:
                text += "\n" + marker + "\n" + "\n".join(
                    "add_dependencies(%s ${catkin_EXPORTED_TARGETS})" % target
                    for target in targets) + "\n"
            return text
        rewrite(source / relative, add_dependencies)


def camera():
    path = WORKSPACES / "realsense_ws/src/realsense-ros/realsense2_camera/CMakeLists.txt"
    def set_opencv(text):
        marker = "# Runtime migration: prefer the OpenCV matching cv_bridge over SDK include paths."
        if marker not in text:
            text = text.replace("# RealSense ROS Node", marker + "\n"
                "find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)\n"
                "include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})\n\n"
                "# RealSense ROS Node")
        return text
    rewrite(path, set_opencv)


if __name__ == "__main__":
    relocate_cv_bridge()
    choices = {"control": control, "camera": camera}
    for name in sys.argv[1:] or choices:
        choices[name]()
