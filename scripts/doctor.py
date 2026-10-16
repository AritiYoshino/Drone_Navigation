#!/usr/bin/env python3
"""Inspect software without starting ROS nodes; --hardware inspects a running chain."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import re
import subprocess
import sys

REPO = Path(__file__).resolve().parent.parent
RESULTS = []


def run(profile, args, timeout=25):
    command = ["bash", "--noprofile", "--norc", "-c",
               'source "$1"; drone_source_profile "$2" || exit; shift 2; exec "$@"',
               "drone-doctor", str(REPO / "scripts/env.sh"), profile] + args
    try:
        return subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              universal_newlines=True, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", "replace")
        return subprocess.CompletedProcess(command, 124,
                                           output + "\nTimed out after {} s".format(timeout))
    except OSError as error:
        return subprocess.CompletedProcess(command, 127, str(error))


def record(name, passed, detail):
    RESULTS.append({"name": name, "passed": bool(passed), "detail": detail})
    print("[{}] {}: {}".format("OK" if passed else "FAIL", name, detail))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware", action="store_true",
                        help="inspect devices and topics after starting the chain")
    args = parser.parse_args()
    RESULTS[:] = []
    paths = run("ros", ["python3", "-c", "import os,json; print(json.dumps({k:v for k,v in os.environ.items() if k.startswith('DRONE_')}))"])
    if paths.returncode:
        record("environment", False, paths.stdout.strip())
        return 1
    env = json.loads(paths.stdout.strip().splitlines()[-1])
    record("ROS", Path(env["DRONE_ROS_ROOT"]).joinpath("setup.bash").is_file(), env["DRONE_ROS_ROOT"])
    try:
        cuda_path = Path(env["DRONE_CUDA_ROOT"]) / "lib64/libcudart.so"
        cuda = ctypes.CDLL(str(cuda_path) if cuda_path.is_file() else "libcudart.so.10.2")
        count, major, minor = ctypes.c_int(), ctypes.c_int(), ctypes.c_int()
        code = cuda.cudaGetDeviceCount(ctypes.byref(count))
        attribute_status = []
        if code == 0 and count.value:
            attribute_status = [cuda.cudaDeviceGetAttribute(ctypes.byref(major), 75, 0),
                                cuda.cudaDeviceGetAttribute(ctypes.byref(minor), 76, 0)]
        record("CUDA", code == 0 and count.value > 0 and attribute_status == [0, 0],
               "devices={}, compute={}.{}, status={}".format(count.value, major.value, minor.value, code))
    except OSError as error:
        record("CUDA", False, str(error))
    targets = [
        ("vins", Path(env["DRONE_VINS_DEVEL"]) / "lib/vins/vins_node", "3.3"),
        ("fast", Path(env["DRONE_FAST_WS"]) / "devel/lib/ego_planner/ego_planner_node", "3.2"),
        ("fast", Path(env["DRONE_FAST_WS"]) / "devel/lib/px4ctrl/px4ctrl_node", None),
        ("fast", Path(env["DRONE_FAST_WS"]) / "devel/lib/px4ctrl/vins_to_px4_odom_node", None),
        ("camera", Path(env["DRONE_RS_WS"]) / "devel/lib/librealsense2_camera.so", "3.2"),
    ]
    for profile, path, expected in targets:
        if not path.is_file():
            record(path.name, False, "missing {}".format(path))
            continue
        result = run(profile, ["ldd", str(path)])
        missing = [line.strip() for line in result.stdout.splitlines() if "not found" in line]
        versions = sorted(set(re.findall(r"libopencv_\w+\.so\.(\d+\.\d+)", result.stdout)))
        passed = result.returncode == 0 and not missing and (expected is None or versions == [expected])
        record(path.name, passed, "; ".join(missing) if missing else
               "libraries resolved; OpenCV={}".format(",".join(versions) or "unused"))
    geoid = Path("/usr/share/GeographicLib/geoids/egm96-5.pgm")
    record("MAVROS geoid", geoid.is_file(), str(geoid))
    for profile, package, expected_root in [
            ("vins", "vins", env["DRONE_VINS_WS"]),
            ("fast", "ego_planner", env["DRONE_FAST_WS"]),
            ("camera", "realsense2_camera", env["DRONE_RS_WS"]),
            ("kalibr", "kalibr", env["DRONE_KALIBR_WS"])]:
        result = run(profile, ["rospack", "find", package])
        record("package " + package, result.returncode == 0 and expected_root in result.stdout,
               result.stdout.strip())
    result = run("kalibr", ["python2", "-c",
                 "import numpy,scipy,matplotlib,igraph,pyx,yaml; print('Python2 scientific/calibration imports passed')"])
    record("Kalibr Python", result.returncode == 0, result.stdout.strip())
    result = run("kalibr", ["python2", "-c",
                 "import aslam_cv,aslam_backend,kalibr_common; print('Kalibr compiled bindings imported')"])
    record("Kalibr bindings", result.returncode == 0, result.stdout.strip())
    if args.hardware:
        hardware_checks(env)
    else:
        print("Hardware checks pending: connect the flight controller/camera, start the chain, then run drone-nav doctor --hardware.")
    report = REPO / ".runtime" / ("hardware-report.json" if args.hardware else "offline-report.json")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps({"hardware_requested": args.hardware, "checks": RESULTS}, indent=2))
    print("Report: {}".format(report))
    return 0 if all(item["passed"] for item in RESULTS) else 1


def hardware_checks(env):
    url = env.get("DRONE_FCU_URL", "/dev/ttyACM0")
    if url.startswith("/"):
        serial = url.split(":", 1)[0]
        record("FCU serial", os.access(serial, os.R_OK | os.W_OK), serial)
    products = []
    for path in Path("/sys/bus/usb/devices").glob("*/product"):
        try:
            value = path.read_text().strip()
            if "RealSense" in value:
                products.append(value)
        except OSError:
            pass
    record("RealSense USB", bool(products), ", ".join(products) or "not detected")
    # Subscribe only: no services, parameter writes, or flight commands.
    code = r'''
import json, time, rospy
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
rospy.init_node('drone_environment_check', anonymous=True, disable_signals=True)
specs = [('/mavros/imu/data_raw', Imu), ('/uav1/camera/infra1/image_rect_raw', Image), ('/uav1/camera/infra2/image_rect_raw', Image), ('/uav1/camera/depth/image_rect_raw', Image), ('/uav1/vins_estimator/imu_propagate', Odometry), ('/mavros/odometry/out', Odometry)]
data = {name: [] for name, _ in specs}
state = [False]
def receive(msg, name):
    data[name].append((time.time(), msg.header.stamp.to_sec()))
subs = [rospy.Subscriber(name, typ, receive, callback_args=name, queue_size=100) for name, typ in specs]
subs.append(rospy.Subscriber('/mavros/state', State, lambda msg: state.__setitem__(0, msg.connected)))
time.sleep(6)
now = rospy.Time.now().to_sec()
out = {'connected': state[0], 'topics': {}}
for name, values in data.items():
    stamps = [v[1] for v in values]
    out['topics'][name] = {'messages': len(values), 'hz': round((len(values)-1)/(values[-1][0]-values[0][0]),1) if len(values)>1 and values[-1][0]>values[0][0] else 0, 'age': round(now-stamps[-1],3) if stamps else None, 'timestamps_increase': all(b>a for a,b in zip(stamps,stamps[1:])) if len(stamps)>1 else False, 'timestamps_nondecrease': all(b>=a for a,b in zip(stamps,stamps[1:])) if len(stamps)>1 else False, 'timestamps_advance': stamps[-1]>stamps[0] if len(stamps)>1 else False}
print(json.dumps(out))
'''
    result = run("ros", ["python2", "-c", code], timeout=15)
    if result.returncode:
        record("ROS hardware topics", False, result.stdout.strip())
        return
    snapshot = json.loads(result.stdout.strip().splitlines()[-1])
    record("MAVROS connected", snapshot["connected"], str(snapshot["connected"]))
    for topic, value in sorted(snapshot["topics"].items()):
        # Timer based odometry bridges may republish the most recent estimate.
        # Accept duplicate stamps there, while rejecting backwards/frozen time.
        increasing = value["timestamps_increase"]
        if topic == "/mavros/odometry/out":
            increasing = value["timestamps_nondecrease"] and value["timestamps_advance"]
        passed = value["messages"] > 1 and increasing and -0.1 <= value["age"] <= 1.0
        record(topic, passed, json.dumps(value, sort_keys=True))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (subprocess.TimeoutExpired, OSError, ValueError, KeyError) as error:
        print("[FAIL] {}".format(error), file=sys.stderr)
        sys.exit(1)
