#!/usr/bin/env python3
"""Exercise startup contracts with fake ROS commands, never ROS or hardware.

Run with: python3 -m unittest discover -s tests -p 'test_drone_nav.py' -v
The launcher is copied into a temporary repository whose path contains spaces.
Its profile setup points exclusively at the mocks below.
"""

import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
MOCK_COMMAND = r'''#!/usr/bin/python3
import json
import os
from pathlib import Path
import subprocess
import sys
import time

name = Path(sys.argv[0]).name
args = sys.argv[1:]
role = name
if args == ['detached']:
    role = 'detached'
elif name == 'roslaunch':
    role = Path(args[0]).stem if args[0].endswith('.launch') else args[1]
elif name == 'rosrun':
    role = args[1]
elif name == 'ros-python':
    role = Path(args[0]).stem

def record(**extra):
    event = dict(command=name, role=role, args=args, pid=os.getpid(),
                 marker=os.environ.get('DRONE_SESSION_ID', ''),
                 profile=os.environ.get('DRONE_ENV_PROFILE', ''),
                 master_ready=Path(os.environ['MOCK_MASTER_READY']).exists())
    event.update(extra)
    fd = os.open(os.environ['MOCK_EVENTS'], os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
    try:
        os.write(fd, (json.dumps(event) + '\n').encode())
    finally:
        os.close(fd)

if name == 'roscore':
    record()
    # Components must wait until the master reports readiness.
    time.sleep(0.1)
    Path(os.environ['MOCK_MASTER_READY']).touch()
elif name == 'rosnode':
    record()
    sys.exit(0 if Path(os.environ['MOCK_MASTER_READY']).exists() else 1)
elif name == 'rostopic':
    record()
    sys.exit(0)
elif role == 'check_vins_gravity':
    record()
    sys.exit(int(os.environ.get('MOCK_GRAVITY_STATUS', '0')))
else:
    record()

if role == os.environ.get('MOCK_DETACH_ROLE'):
    # roslaunch nodes can escape the launcher's process group with setsid().
    subprocess.Popen([sys.executable, __file__, 'detached'], start_new_session=True)
if args == ['detached']:
    pass
elif role == os.environ.get('MOCK_EXIT_ROLE'):
    sys.exit(7)
elif os.environ.get('MOCK_ONESHOT') == '1':
    sys.exit(0)
while True:
    time.sleep(0.1)
'''


def running(pid):
    """A zombie is already stopped, even if init has not reaped it yet."""
    try:
        return Path('/proc/{}/stat'.format(pid)).read_text().rsplit(') ', 1)[1].split()[0] != 'Z'
    except (OSError, IndexError):
        return False


class DroneNavTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='drone nav tests ')
        self.repo = Path(self.temporary.name) / 'repository with spaces'
        self.scripts = self.repo / 'scripts'
        (self.scripts / 'home').mkdir(parents=True)
        for name in ('drone_nav.sh', 'env.sh', 'nav_env.sh', 'prepare_vins_config.py', 'stop_session.py'):
            shutil.copy2(ROOT / 'scripts' / name, self.scripts / name)
        for name in ('vi_gpu.sh', 'test.sh', 'vins_quick_start_gpu.sh', 'start_drone.sh', 'takeoff.sh'):
            shutil.copy2(ROOT / 'scripts/home' / name, self.scripts / 'home' / name)
        self.mock_bin = self.repo / 'mock commands'
        self.mock_bin.mkdir()
        mock = self.mock_bin / 'mock-command'
        mock.write_text(MOCK_COMMAND)
        mock.chmod(0o755)
        for name in ('roscore', 'rosnode', 'roslaunch', 'rosrun', 'rostopic', 'ros-python'):
            (self.mock_bin / name).symlink_to(mock.name)
        # This is deliberately a fixture, not the real environment setup.
        (self.scripts / 'local_env.sh').write_text(
            'export PATH="$MOCK_BIN:/usr/bin:/bin"\n'
            'export DRONE_ENV_PROFILE="$1"\n'
        )
        (self.repo / 'config').mkdir()
        shutil.copy2(ROOT / 'config/drone.env', self.repo / 'config/drone.env')
        self.runtime = self.repo / 'runtime files'
        self.ready = self.repo / 'master ready'
        self.events_path = self.repo / 'events.jsonl'
        self.config = self.repo / 'calibration files' / 'stereo config.yaml'
        self.config.parent.mkdir()
        self.config.write_text(
            '%YAML:1.0\ncam0_calib: "left camera.yaml"\n'
            'cam1_calib: "right camera.yaml"\noutput_path: old\n'
            'pose_graph_save_path: old\n'
        )
        for name in ('left camera.yaml', 'right camera.yaml'):
            (self.config.parent / name).write_text('%YAML:1.0\n')
        self.env = {
            key: value for key, value in os.environ.items()
            if not key.startswith(('DRONE_', 'ROS_', 'MOCK_', 'LD_'))
            and key not in ('DISPLAY', 'PYTHONPATH', 'CMAKE_PREFIX_PATH')
        }
        self.env.update({
            'DRONE_REPO_ROOT': str(self.repo),
            'DRONE_RUNTIME_DIR': str(self.runtime),
            'DRONE_VINS_CONFIG': str(self.config),
            'DRONE_ROS_PYTHON': str(self.mock_bin / 'ros-python'),
            'DRONE_TOPIC_TIMEOUT': '3',
            'ROS_MASTER_URI': 'http://localhost:11311',
            'MOCK_BIN': str(self.mock_bin),
            'MOCK_EVENTS': str(self.events_path),
            'MOCK_MASTER_READY': str(self.ready),
        })
        for variable, directory in (
            ('DRONE_RS_WS', 'camera workspace'),
            ('DRONE_VINS_WS', 'vins workspace'),
            ('DRONE_FAST_WS', 'planner workspace'),
        ):
            workspace = self.repo / directory
            (workspace / 'devel').mkdir(parents=True)
            (workspace / 'devel/setup.bash').write_text(':\n')
            self.env[variable] = str(workspace)
        self.processes = []

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=12)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=3)
        # Also reap a mock if an assertion interrupted the normal cleanup test.
        for event in self.events():
            if running(event['pid']):
                try:
                    command_line = Path('/proc/{}/cmdline'.format(event['pid'])).read_bytes()
                    if str(self.mock_bin).encode() in command_line:
                        os.kill(event['pid'], signal.SIGKILL)
                except OSError:
                    pass
        self.temporary.cleanup()

    def events(self):
        if not self.events_path.exists():
            return []
        return [json.loads(line) for line in self.events_path.read_text().splitlines() if line]

    def clear_events(self):
        if self.events_path.exists():
            self.events_path.unlink()

    def command(self, *arguments, script='drone_nav.sh'):
        return ['bash', '--noprofile', '--norc', str(self.scripts / script)] + list(arguments)

    def run_nav(self, *arguments, extra_env=None, script='drone_nav.sh', timeout=8):
        env = dict(self.env, **(extra_env or {}))
        return subprocess.run(self.command(*arguments, script=script), env=env,
                              universal_newlines=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=timeout)

    def start_nav(self, *arguments, extra_env=None):
        process = subprocess.Popen(self.command(*arguments), env=dict(self.env, **(extra_env or {})),
                                   universal_newlines=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def await_event(self, process, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for event in self.events():
                if predicate(event):
                    return event
            if process.poll() is not None:
                output = process.communicate()[0]
                self.fail('Launcher exited before expected event: {}'.format(output))
            time.sleep(0.03)
        self.fail('Expected mock command was not called; events: {}'.format(self.events()))

    def stop_nav(self, process):
        process.terminate()
        output = process.communicate(timeout=12)[0]
        self.assertEqual(process.returncode, 143, output)
        return output

    @staticmethod
    def components(output):
        return re.findall(r'^\[([a-z-]+) / [a-z]+\]', output, re.M)

    def test_dry_runs_preserve_reference_vins_and_start_flows(self):
        for action, expected in (
            ('vins', ['master', 'mavros', 'imu-rate', 'camera', 'vins']),
            ('start', ['master', 'mavros', 'imu-rate', 'camera', 'vins', 'bridge', 'planner']),
            ('quick', ['master', 'mavros', 'imu-rate', 'camera', 'vins', 'bridge', 'planner']),
        ):
            with self.subTest(action=action):
                result = self.run_nav(action, '--dry-run', '--headless')
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertEqual(self.components(result.stdout), expected)
        self.assertEqual(self.events(), [])
        self.assertFalse(self.runtime.exists())

    def test_legacy_wrappers_map_to_reference_flows(self):
        for script, arguments, expected in (
            ('vi_gpu.sh', [], 'vins'),
            ('test.sh', [], 'vins'),
            ('vins_quick_start_gpu.sh', [], 'start'),
            ('start_drone.sh', [], 'vins'),
            ('start_drone.sh', ['--mode', 'full'], 'start'),
        ):
            with self.subTest(script=script, arguments=arguments):
                result = self.run_nav(*(arguments + ['--dry-run', '--headless']),
                                      script='home/' + script)
                canonical = self.run_nav(expected, '--dry-run', '--headless')
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertEqual(result.stdout, canonical.stdout)
        self.assertEqual(self.events(), [])

    def test_controller_goals_and_takeoff_are_explicit_commands(self):
        cases = (
            (['controller'], 'controller', ['roslaunch', 'px4ctrl', 'run_ctrl.launch']),
            (['component', 'goals'], 'goals', ['rosrun', 'ego_planner', 'realtime_goal.py']),
            (['takeoff'], 'takeoff', ['rostopic', 'pub', '-1', '/px4ctrl/takeoff_land',
                                     'quadrotor_msgs/TakeoffLand', 'takeoff_land_cmd: 1']),
        )
        for arguments, component, expected in cases:
            with self.subTest(component=component):
                result = self.run_nav(*(arguments + ['--dry-run']))
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertEqual(self.components(result.stdout), [component])
                self.assertEqual(shlex.split(result.stdout.split('] ', 1)[1]), expected)
        self.assertEqual(self.events(), [])

    def test_gravity_failure_blocks_bridge_and_controller(self):
        for component in ('bridge', 'controller'):
            with self.subTest(component=component):
                self.clear_events()
                result = self.run_nav('component', component,
                                      extra_env={'MOCK_GRAVITY_STATUS': '1', 'MOCK_ONESHOT': '1'})
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('gravity check failed', result.stdout)
                events = self.events()
                self.assertEqual([event['role'] for event in events],
                                 ['rostopic', 'rostopic', 'check_vins_gravity'])
                self.assertEqual([event['args'][-1] for event in events[:2]],
                                 ['/mavros/imu/data_raw', '/uav1/vins_estimator/imu_propagate'])

    def test_gravity_success_precedes_bridge_or_controller_launch(self):
        for component, launch in (('bridge', 'vins_to_px4_odom.launch'),
                                  ('controller', 'run_ctrl.launch')):
            with self.subTest(component=component):
                self.clear_events()
                result = self.run_nav('component', component, extra_env={'MOCK_ONESHOT': '1'})
                self.assertEqual(result.returncode, 0, result.stdout)
                events = self.events()
                self.assertEqual(events[-2]['role'], 'check_vins_gravity')
                self.assertEqual(events[-1]['args'], ['px4ctrl', launch])
                self.assertEqual(events[-1]['profile'], 'control')

    def test_paths_and_option_values_with_spaces_reach_the_ros_commands(self):
        fcu = '/dev/flight controller:921600'
        result = self.run_nav('component', 'mavros', '--fcu-url', fcu,
                              extra_env={'MOCK_ONESHOT': '1'})
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.events()[-1]['args'][1], 'fcu_url:=' + fcu)
        legacy = self.run_nav('--mode', 'mavros', '--fcu-url', fcu, '--dry-run',
                              script='home/start_drone.sh')
        self.assertEqual(legacy.returncode, 0, legacy.stdout)
        mavros_line = next(line for line in legacy.stdout.splitlines() if line.startswith('[mavros /'))
        self.assertIn('fcu_url:=' + fcu, shlex.split(mavros_line.split('] ', 1)[1]))
        serial = 'camera serial 123'
        result = self.run_nav('component', 'camera', '--serial', serial,
                              extra_env={'MOCK_ONESHOT': '1'})
        self.assertEqual(result.returncode, 0, result.stdout)
        camera = self.events()[-1]
        self.assertEqual(camera['args'][0], self.env['DRONE_RS_WS'] +
                         '/src/realsense-ros/realsense2_camera/launch/rs_camera_ego.launch')
        self.assertIn('serial_no:=' + serial, camera['args'])
        self.assertEqual(camera['profile'], 'camera')
        original = self.config.read_bytes()
        result = self.run_nav('component', 'vins', '--vins-config', str(self.config),
                              extra_env={'MOCK_ONESHOT': '1'})
        self.assertEqual(result.returncode, 0, result.stdout)
        vins = self.events()[-1]
        runtime_config = self.runtime / 'vins_runtime.yaml'
        self.assertEqual(vins['args'], ['vins', 'vins_node', str(runtime_config), '__ns:=/uav1'])
        self.assertEqual(vins['profile'], 'vins')
        self.assertEqual(self.config.read_bytes(), original)
        content = runtime_config.read_text()
        for key, calibration in (('cam0_calib', 'left camera.yaml'), ('cam1_calib', 'right camera.yaml')):
            relative = json.loads(re.search(r'^' + key + r': (.+)$', content, re.M).group(1))
            self.assertEqual((self.runtime / relative).resolve(), self.config.parent / calibration)

    def test_exported_values_and_cli_options_override_configuration_files(self):
        (self.repo / 'runtime').mkdir()
        (self.repo / 'runtime/host.env').write_text('DRONE_FCU_URL="host default"\n')
        with (self.repo / 'config/drone.env').open('a') as stream:
            stream.write('\nDRONE_FCU_URL="project default"\n')
        (self.repo / 'config/local.env').write_text('DRONE_FCU_URL="local default"\n')
        for arguments, extra, expected in (
            ([], {}, 'local default'),
            ([], {'DRONE_FCU_URL': 'exported override'}, 'exported override'),
            (['--fcu-url', 'cli override'], {'DRONE_FCU_URL': 'exported override'}, 'cli override'),
        ):
            with self.subTest(expected=expected):
                result = self.run_nav('component', 'mavros', *arguments,
                                      extra_env=dict(extra, MOCK_ONESHOT='1'))
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertEqual(self.events()[-1]['args'][1], 'fcu_url:=' + expected)

    def test_headless_starts_one_ready_master_and_cleans_detached_children(self):
        unrelated = subprocess.Popen(['/usr/bin/python3', '-c', 'import time; time.sleep(60)'],
                                     start_new_session=True)
        self.processes.append(unrelated)
        process = self.start_nav('start', '--headless',
                                 extra_env={'MOCK_DETACH_ROLE': 'mavros_px4'})
        self.await_event(process, lambda event: event['role'] == 'vins_node')
        self.await_event(process, lambda event: event['role'] == 'single_run_in_exp.launch')
        detached = self.await_event(process, lambda event: event['args'] == ['detached'])
        events = self.events()
        masters = [event for event in events if event['command'] == 'roscore']
        self.assertEqual(len(masters), 1)
        launched = [event for event in events if event['command'] not in ('rosnode', 'roscore')]
        self.assertTrue(all(event['master_ready'] for event in launched))
        self.assertTrue(detached['marker'].startswith('drone-nav.'))
        self.assertEqual({event['marker'] for event in launched}, {masters[0]['marker']})
        self.stop_nav(process)
        self.assertFalse(running(detached['pid']), 'Detached roslaunch child survived session cleanup')
        self.assertTrue(all(not running(event['pid']) for event in events
                            if event['marker'] == masters[0]['marker']))
        self.assertIsNone(unrelated.poll(), 'Cleanup stopped an unrelated process')

    def test_headless_reuses_existing_local_or_remote_master(self):
        self.ready.touch()
        for uri in ('http://localhost:11311', 'http://192.0.2.10:11311'):
            with self.subTest(uri=uri):
                self.clear_events()
                process = self.start_nav('vins', '--headless', extra_env={'ROS_MASTER_URI': uri})
                self.await_event(process, lambda event: event['role'] == 'vins_node')
                self.stop_nav(process)
                self.assertNotIn('roscore', [event['command'] for event in self.events()])
                self.assertTrue(self.ready.exists())

    def test_unavailable_remote_master_does_not_start_any_local_ros_process(self):
        result = self.run_nav('start', '--headless', extra_env={
            'ROS_MASTER_URI': 'http://192.0.2.10:11311', 'DRONE_TOPIC_TIMEOUT': '1',
        })
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('ROS master did not come up', result.stdout)
        self.assertTrue(self.events())
        self.assertEqual({event['command'] for event in self.events()}, {'rosnode'})

    def test_component_failure_stops_the_headless_session(self):
        process = self.start_nav('start', '--headless',
                                 extra_env={'MOCK_EXIT_ROLE': 'single_run_in_exp.launch'})
        output = process.communicate(timeout=12)[0]
        self.assertEqual(process.returncode, 7, output)
        marked = [event for event in self.events() if event['marker']]
        self.assertTrue(marked)
        self.assertTrue(all(not running(event['pid']) for event in marked), self.events())


if __name__ == '__main__':
    unittest.main()
