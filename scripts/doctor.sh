#!/usr/bin/env bash
# Offline checks only. This never starts ROS nodes or sends vehicle commands.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
exec /usr/bin/python3 - "$PROJECT_ROOT" "$@" <<'PY'
import argparse
import grp
import glob
import json
import os
import platform
import re
import subprocess
import sys

parser = argparse.ArgumentParser(description='无人机软件离线检查；不会启动节点或发送飞控指令。')
parser.add_argument('--json', action='store_true', help='输出机器可读的 JSON')
parser.add_argument('--require-hardware', action='store_true', help='接入硬件后，将缺设备计为失败')
args = parser.parse_args(sys.argv[2:])
root = os.path.realpath(sys.argv[1])
checks = []

def record(status, name, detail):
    checks.append({'status': status, 'name': name, 'detail': detail})

def run(command, env=None, timeout=25, input_text=None):
    try:
        proc = subprocess.run(command, env=env, input=input_text,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, timeout=timeout)
        return proc.returncode, proc.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)

def profile_env(profile):
    command = ['bash', '--noprofile', '--norc', '-c',
        'source "$1" "$2" >/dev/null && env -0', 'drone-doctor',
        os.path.join(root, 'scripts', 'env.sh'), profile]
    try:
        proc = subprocess.run(command, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=25)
    except (OSError, subprocess.TimeoutExpired) as exc:
        record('FAIL', profile + ' 环境', str(exc))
        return None
    if proc.returncode:
        record('FAIL' if profile != 'kalibr' else 'WARN', profile + ' 环境',
            proc.stderr.decode(errors='replace').strip())
        return None
    result = {}
    for entry in proc.stdout.decode(errors='surrogateescape').split('\0'):
        if '=' in entry:
            key, value = entry.split('=', 1)
            result[key] = value
    record('PASS', profile + ' 环境', '已独立加载')
    return result

record('PASS' if platform.machine() == 'aarch64' else 'FAIL', 'CPU 架构', platform.machine())
try:
    with open('/proc/device-tree/model', 'r') as stream:
        model = stream.read().replace('\0', '').strip()
    record('PASS', '计算平台', model)
except OSError:
    record('WARN', '计算平台', '未发现 Jetson 设备树')

rc, output = run(['/usr/local/cuda-10.2/bin/nvcc', '--version'])
match = re.search(r'release ([0-9.]+)', output)
record('PASS' if rc == 0 else 'FAIL', 'CUDA 编译工具',
    match.group(1) if match else output)

profiles = {}
packages = {
    'base': ['roscpp', 'mavros', 'mavros_msgs'],
    'camera': ['realsense2_camera'],
    'vins': ['vins', 'camera_models', 'cv_bridge'],
    'control': ['ego_planner', 'px4ctrl', 'quadrotor_msgs'],
    'kalibr': ['kalibr'],
}
for profile, names in packages.items():
    env = profile_env(profile)
    profiles[profile] = env
    if env is None:
        continue
    for name in names:
        rc, path = run(['rospack', 'find', name], env)
        record('PASS' if rc == 0 else ('WARN' if profile == 'kalibr' else 'FAIL'),
            profile + ' / ' + name, path)

def check_elf(profile, name, relative):
    env = profiles.get(profile)
    if env is None:
        return
    filename = os.path.join(root, 'runtime', 'workspaces', relative)
    if not os.path.isfile(filename):
        record('FAIL', name + ' 可执行/共享库', '尚未构建：' + filename)
        return
    rc, output = run(['ldd', filename], env)
    missing = [line.strip() for line in output.splitlines() if 'not found' in line]
    if rc or missing:
        record('FAIL', name + ' 动态库', '; '.join(missing) or output)
        return
    record('PASS', name + ' 动态库', '全部解析成功')
    versions = sorted(set(re.findall(r'libopencv_core\.so\.([0-9]+\.[0-9]+)', output)))
    if versions:
        record('PASS' if len(versions) == 1 else 'FAIL', name + ' OpenCV ABI', ', '.join(versions))
    if profile == 'vins':
        gpu_libs = ['libopencv_cudaoptflow', 'libopencv_cudaimgproc', 'libopencv_cudaarithm']
        absent = [name for name in gpu_libs if name not in output]
        record('FAIL' if absent else 'PASS', 'VINS GPU 链接',
            '缺少：' + ', '.join(absent) if absent else 'CUDA OpenCV 光流/图像处理/计算库已链接')

for item in [
    ('camera', 'RealSense', 'realsense_ws/devel/lib/librealsense2_camera.so'),
    ('vins', 'GPU VINS', 'vinsf_gpu_ws/devel/lib/vins/vins_node'),
    ('control', 'EGO', 'Fast-Drone-250-master/devel/lib/ego_planner/ego_planner_node'),
    ('control', 'PX4 控制器', 'Fast-Drone-250-master/devel/lib/px4ctrl/px4ctrl_node'),
    ('control', 'VINS → PX4 里程计桥', 'Fast-Drone-250-master/devel/lib/px4ctrl/vins_to_px4_odom_node'),
]:
    check_elf(*item)

goal_script = os.path.join(root, 'runtime', 'workspaces', 'Fast-Drone-250-master',
    'src', 'planner', 'plan_manage', 'scripts', 'realtime_goal.py')
try:
    with open(goal_script, 'rb') as stream:
        goal_bytes = stream.read()
    goal_ok = goal_bytes.startswith(b'#!') and b'\r\n' not in goal_bytes and os.access(goal_script, os.X_OK)
except OSError:
    goal_ok = False
record('PASS' if goal_ok else 'FAIL', 'EGO 目标脚本格式/执行权限',
    'LF 换行和执行权限正常' if goal_ok else goal_script + ' 需要恢复 LF 换行及执行权限')
if goal_ok and profiles.get('control'):
    # This script parses --help and exits before rospy.init_node.
    rc, output = run(['/usr/bin/python2', goal_script, '--help'], profiles['control'])
    record('PASS' if rc == 0 and '--topic' in output else 'FAIL', 'EGO 目标脚本加载',
        '--help 解析成功（未启动节点）' if rc == 0 else output)

base_env = profiles.get('base') or os.environ.copy()
rc, output = run(['GeoidEval', '-n', 'egm96-5', '-l'], base_env, input_text='0 0\n')
record('PASS' if rc == 0 else 'FAIL', 'MAVROS EGM96 数据', output)

gpu_check = os.path.join(root, 'runtime', 'local', 'bin', 'drone_gpu_check')
if os.path.isfile(gpu_check) and os.access(gpu_check, os.X_OK):
    gpu_env = profiles.get('vins') or base_env
    rc, output = run([gpu_check], gpu_env, timeout=45)
    record('PASS' if rc == 0 else 'FAIL', 'GPU 实际计算', output)
else:
    record('FAIL', 'GPU 实际计算', '尚未构建 runtime/local/bin/drone_gpu_check')

ceres_check = os.path.join(root, 'runtime', 'local', 'bin', 'drone_ceres_check')
if os.path.isfile(ceres_check) and os.access(ceres_check, os.X_OK):
    rc, output = run([ceres_check], base_env, timeout=45)
    record('PASS' if rc == 0 else 'FAIL', 'Ceres CUDA 数值求解', output)
else:
    record('FAIL', 'Ceres CUDA 数值求解', '构建 Ceres 后运行 scripts/check_ceres_cuda.sh')

kalibr_env = profiles.get('kalibr')
if kalibr_env:
    rc, output = run(['/usr/bin/python2', '-B', '-c',
        'import numpy,scipy,matplotlib,yaml,igraph,pyx,cv2; '
        'assert cv2.__version__.startswith("3.2."), cv2.__version__; '
        'print("Python 2 标定依赖导入成功；OpenCV " + cv2.__version__)'], kalibr_env)
    record('PASS' if rc == 0 else 'WARN', 'Kalibr Python 2 依赖', output)
    rc, output = run(['/usr/bin/python2', '-B', '-c',
        'import numpy as np, scipy.linalg, aslam_cameras_april as april; '
        'from cv_bridge import CvBridge; '
        'a=np.array([[4.,1.],[1.,3.]]); b=np.array([1.,2.]); '
        'x=scipy.linalg.solve(a,b); assert np.linalg.norm(a.dot(x)-b)<1e-12; '
        'target=april.GridCalibrationTargetAprilgrid(6,6,.088,.3); '
        'image=np.arange(15,dtype=np.uint8).reshape(3,5); bridge=CvBridge(); '
        'message=bridge.cv2_to_imgmsg(image,encoding="mono8"); '
        'color=bridge.imgmsg_to_cv2(message,desired_encoding="bgr8"); '
        'assert color.shape==(3,5,3) and np.array_equal(color[:,:,0],image); '
        'print("SciPy 求解、AprilGrid C++ 对象及 cv_bridge 图像转换验证通过")'], kalibr_env, timeout=45)
    record('PASS' if rc == 0 else 'WARN', 'Kalibr 数值库', output)
    rc, output = run(['rosrun', 'kalibr', 'kalibr_calibrate_cameras', '--help'], kalibr_env, timeout=45)
    # Legacy Kalibr catches argparse's successful SystemExit and returns 2.
    help_ok = rc in (0, 2) and 'usage:' in output.lower() and '--models' in output and 'Traceback' not in output
    record('PASS' if help_ok else 'WARN', 'Kalibr 工具加载',
        '标定工具完整帮助加载成功' if help_ok else output[-1500:])
    pdf = os.path.join(root, 'runtime', 'checks', 'kalibr_aprilgrid_test.pdf')
    try:
        with open(pdf, 'rb') as stream:
            valid_pdf = stream.read(5) == b'%PDF-'
        valid_pdf = valid_pdf and os.path.getsize(pdf) > 1000
    except OSError:
        valid_pdf = False
    record('PASS' if valid_pdf else 'WARN', 'Kalibr PDF 实际生成记录',
        pdf if valid_pdf else '尚无 AprilGrid PDF 验证产物；可运行 kalibr_create_target_pdf 生成诊断板')

camera_count = 0
for entry in glob.glob('/sys/bus/usb/devices/*'):
    try:
        with open(os.path.join(entry, 'product')) as stream:
            product = stream.read()
        if 'realsense' in product.lower():
            camera_count += 1
    except OSError:
        pass
pending = 'FAIL' if args.require_hardware else 'PENDING'
record('PASS' if camera_count else pending, 'RealSense 硬件',
    '检测到 %d 个 RealSense USB 设备' % camera_count if camera_count else '待插入相机后测试 USB 3、图像和时间戳')
serials = sorted(set(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')))
if serials:
    writable = [path for path in serials if os.access(path, os.R_OK | os.W_OK)]
    record('PASS' if writable else 'FAIL', '飞控串口',
        '发现 %d 个候选串口，可读写 %d 个；尚需确认飞控身份' % (len(serials), len(writable)))
else:
    record(pending, '飞控硬件', '待插入飞控后确认串口及 MAVROS 通信')
groups = run(['id', '-Gn'])[1].split()
if 'dialout' not in groups:
    try:
        configured = os.environ.get('USER', '') in grp.getgrnam('dialout').gr_mem
    except KeyError:
        configured = False
    record('WARN', '串口组权限', '当前用户不在 dialout；插入飞控后检查设备可读写，管理员配置见部署说明')
    if configured:
        checks[-1]['detail'] = 'dialout 已配置；当前登录会话尚未生效，请重新登录后检查飞控串口可读写'

failures = sum(item['status'] == 'FAIL' for item in checks)
warnings = sum(item['status'] == 'WARN' for item in checks)
summary = {'software_and_requested_hardware_passed': failures == 0,
    'failures': failures, 'warnings': warnings,
    'pending_hardware': sum(item['status'] == 'PENDING' for item in checks)}
if args.json:
    print(json.dumps({'summary': summary, 'checks': checks}, ensure_ascii=False, indent=2))
else:
    print('离线检查：不会启动 ROS 节点、解锁或发送起飞命令。')
    for item in checks:
        print('[{status}] {name}: {detail}'.format(**item))
    print('检查完成：失败 %d，提醒 %d，待接硬件 %d。' %
          (failures, warnings, summary['pending_hardware']))
sys.exit(1 if failures else 0)
PY
