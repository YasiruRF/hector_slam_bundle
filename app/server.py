#!/usr/bin/env python3
"""Web dashboard for running Hector SLAM + Nav2 on a TurtleBot 4.

  python3 server.py          # http://localhost:8080 (PORT, DATA_DIR env vars)

Process manager + HTTP/WebSocket front for the browser UI (static/index.html):
  - robot link settings (domain, RMW, namespace, ...) saved in DATA_DIR/settings.json
  - the connection-check pipeline (gates Start)
  - the SLAM + Nav2 launch, the cmd_vel relay and rosbag as managed processes with live logs
  - ros_helper.py (rclpy) for live map/pose/scan, teleop, goals and e-stop
  - evidence: saved map, screenshots, video, rosbag, trajectory, zipped results
"""
import asyncio
import json
import os
import re
import signal
import time
import zipfile
from collections import deque
from pathlib import Path

from aiohttp import WSMsgType, web

APP = Path(__file__).resolve().parent
DATA = Path(os.environ.get('DATA_DIR', '/data'))
DATA.mkdir(parents=True, exist_ok=True)
(DATA / 'results').mkdir(exist_ok=True)
SETTINGS_FILE = DATA / 'settings.json'
DEFAULTS = {
    'namespace': '', 'domain_id': 0, 'rmw': 'rmw_fastrtps_cpp', 'robot_ip': '',
    'discovery_server': '', 'cmd_vel_mode': 'auto',
    'max_lin': 0.2, 'max_ang': 0.8, 'scan_watchdog': True,
    # 'real' = the TurtleBot 4 on the network, 'gazebo' / 'webots' = a simulation started from the dashboard.
    # The simulation gets its own ROS domain so it can never move a real robot.
    'show_windows': True,            # open the Gazebo GUI and RViz (needs a display in the container)
    'target': 'real', 'sim_world': 'maze', 'sim_model': 'standard', 'sim_domain_id': 42,
    'sim_render': 'software',      # software = CPU rendering (correct lidar, slower); auto = GPU if /dev/dri exists (lidar is broken on the GPU path tested)
}
WORLDS = {'gazebo': ('maze', 'warehouse', 'depot'), 'webots': ('hector_slam_test',)}
RMWS = ('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp')


def load_settings():
    s = dict(DEFAULTS)
    try:
        s.update({k: v for k, v in json.loads(SETTINGS_FILE.read_text()).items() if k in DEFAULTS})
    except Exception:
        pass
    return s


class Proc:
    """A child process with a merged, line-buffered log that is streamed to the browser."""

    def __init__(self, name, app):
        self.name, self.app = name, app
        self.p = None
        self.log = deque(maxlen=1500)

    @property
    def running(self):
        return self.p is not None and self.p.returncode is None

    async def start(self, argv, on_line=None, stdin=False):
        self.p = await asyncio.create_subprocess_exec(
            *argv, env=self.app.env(), start_new_session=True,
            stdin=asyncio.subprocess.PIPE if stdin else asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, limit=2 ** 24)
        self.on_line = on_line
        asyncio.create_task(self.pump(self.p))
        await self.app.send_procs()

    async def pump(self, p):
        async for raw in p.stdout:
            line = raw.decode(errors='replace').rstrip()
            if self.on_line and self.on_line(line):
                continue
            self.add(line)
        await p.wait()
        self.add(f'[exited with code {p.returncode}]')
        await self.app.send_procs()

    def add(self, line):
        line = re.sub(r'\x1b\[[0-9;]*m', '', line)
        self.log.append(line)
        asyncio.create_task(self.app.broadcast({'type': 'log', 'proc': self.name, 'line': line}))

    def write(self, obj):
        if self.running and self.p.stdin:
            self.p.stdin.write((json.dumps(obj) + '\n').encode())

    async def stop(self, timeout=8.0):
        if not self.running:
            return
        os.killpg(self.p.pid, signal.SIGINT)
        try:
            await asyncio.wait_for(self.p.wait(), timeout)
        except asyncio.TimeoutError:
            os.killpg(self.p.pid, signal.SIGKILL)
            await self.p.wait()


class App:
    def __init__(self):
        self.settings = load_settings()
        self.clients = set()
        self.procs = {n: Proc(n, self) for n in ('helper', 'slam', 'relay', 'bag', 'map_saver', 'sim', 'sim_robot', 'sim_gui')}
        self.sim_state = 'stopped'        # stopped | starting | ready
        self.cache = {}                  # last message of each type, replayed to new browsers
        self.preflight = {'steps': [], 'ok': False, 'ts': 0, 'stamper_needed': False}
        self.probe_waiters = {}
        self.probe_id = 0
        self.session = time.strftime('%Y%m%d_%H%M%S')
        self.alerted = False

    # ---- plumbing ----
    @property
    def sim(self):
        return self.settings['target'] in ('gazebo', 'webots')

    @property
    def gui_available(self):
        return bool(os.environ.get('DISPLAY')) and os.path.exists('/tmp/.X11-unix')

    @property
    def windows(self):
        return self.settings['show_windows'] and self.gui_available

    @property
    def ns(self):
        n = '' if self.sim else self.settings['namespace'].strip('/')
        return f'/{n}' if n else ''

    def env(self):
        s = self.settings
        e = dict(os.environ, ROS_DOMAIN_ID=str(s['sim_domain_id'] if self.sim else s['domain_id']),
                 RMW_IMPLEMENTATION=s['rmw'], ROS_LOCALHOST_ONLY='0')
        if s['discovery_server'] and not self.sim:
            e['ROS_DISCOVERY_SERVER'] = s['discovery_server']
            e['ROS_SUPER_CLIENT'] = 'True'
        else:
            e.pop('ROS_DISCOVERY_SERVER', None)
        return e

    @property
    def session_dir(self):
        d = DATA / 'results' / self.session
        d.mkdir(parents=True, exist_ok=True)
        return d

    async def broadcast(self, msg):
        if msg['type'] in ('status', 'pose', 'map', 'scan', 'procs', 'settings'):
            self.cache[msg['type']] = msg
        data = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send_str(data)
            except Exception:
                self.clients.discard(ws)

    async def send_procs(self):
        await self.broadcast(self.procs_msg())

    def procs_msg(self):
        return {'type': 'procs', 'running': {n: p.running for n, p in self.procs.items()},
                'session': self.session, 'sim_state': self.sim_state, 'gui_available': self.gui_available}

    # ---- helper (rclpy) ----
    async def start_helper(self):
        await self.procs['helper'].stop(3)
        s = self.settings
        argv = ['python3', str(APP / 'ros_helper.py'), '--namespace', self.ns]
        if self.sim:
            argv.append('--sim-time')
        await self.procs['helper'].start(argv, on_line=self.on_helper_line, stdin=True)

    def on_helper_line(self, line):
        try:
            m = json.loads(line)
        except ValueError:
            return False                       # plain text (errors): goes to the log
        if m['type'] == 'toast':
            self.procs['helper'].add(f"[{m['level']}] {m['text']}")
        if m['type'] == 'probe_result':
            fut = self.probe_waiters.pop(m['id'], None)
            if fut and not fut.done():
                fut.set_result(m)
            return True
        asyncio.create_task(self.on_helper_msg(m))
        return True

    async def on_helper_msg(self, m):
        await self.broadcast(m)
        if m['type'] == 'status':
            await self.watchdog(m)
        elif m['type'] == 'traj_done':
            self.procs['helper'].add(f"trajectory saved: {m['path']} ({m['length_m']} m)")

    async def watchdog(self, st):
        """Lost lidar while something can move the robot: stop it and tell the operator."""
        moving = self.procs['slam'].running or st.get('nav') == 'navigating'
        lost = moving and self.settings['scan_watchdog'] and (st['scan_age'] is None or st['scan_age'] > 3)
        if lost and not self.alerted:
            self.alerted = True
            self.procs['helper'].write({'cmd': 'estop'})
            await self.broadcast({'type': 'alert', 'text': 'Lidar /scan stopped: robot stopped, goals cancelled. '
                                  'Check the robot link, then Re-run checks.'})
        elif not lost:
            self.alerted = False

    async def probe(self):
        self.probe_id += 1
        fut = asyncio.get_running_loop().create_future()
        self.probe_waiters[self.probe_id] = fut
        self.procs['helper'].write({'cmd': 'probe', 'id': self.probe_id})
        return await asyncio.wait_for(fut, 25)

    # ---- connection-check pipeline ----
    async def run_preflight(self):
        s = self.settings
        steps = []

        async def add(sid, title, status, detail, hint=''):
            step = {'id': sid, 'title': title, 'status': status, 'detail': detail, 'hint': hint}
            steps.append(step)
            await self.broadcast({'type': 'preflight', 'step': step})

        await self.broadcast({'type': 'preflight_start'})
        if self.sim:
            await add('settings', 'ROS settings', 'ok' if self.sim_state == 'ready' else 'fail',
                      f"{s['target'].capitalize()} simulation, isolated domain {s['sim_domain_id']}, {s['rmw']}",
                      '' if self.sim_state == 'ready' else 'Start the simulation first (Simulation panel).')
        else:
            await add('settings', 'ROS settings', 'ok',
                      f"domain {s['domain_id']}, {s['rmw']}, namespace '{s['namespace'] or '(none)'}'"
                      + (f", discovery server {s['discovery_server']}" if s['discovery_server'] else ''))

        if self.sim:
            await add('ping', 'Robot reachable', 'ok', 'simulated robot, no network involved')
        elif s['robot_ip']:
            p = await asyncio.create_subprocess_exec('ping', '-c', '2', '-W', '2', s['robot_ip'],
                                                     stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            out = (await p.communicate())[0].decode()
            m = re.search(r'= [\d.]+/([\d.]+)/', out)
            if p.returncode == 0:
                await add('ping', 'Robot reachable', 'ok', f"{s['robot_ip']} answers, {m.group(1)} ms" if m else 'ok')
            else:
                await add('ping', 'Robot reachable', 'fail', f"no answer from {s['robot_ip']}",
                          'Check Wi-Fi / Ethernet, that the robot is on, and the IP address.')
        else:
            await add('ping', 'Robot reachable', 'warn', 'no robot IP set, skipped', 'Enter it in Settings to test the network.')

        if not self.procs['helper'].running:
            await self.start_helper()
            await asyncio.sleep(1.5)
        try:
            pr = await self.probe()
        except Exception as e:
            await add('dds', 'ROS 2 discovery', 'fail', f'ROS helper did not answer ({e!r})', 'See the "helper" log.')
            return await self.finish_preflight(steps)

        ours = {'slam_nav', 'hector_mapping', 'frontier_explorer', 'cmd_vel_stamper'}
        nodes = [n for n in pr['nodes']]
        if not nodes:
            await add('dds', 'ROS 2 discovery', 'fail', 'no ROS nodes visible',
                      'Same ROS_DOMAIN_ID and RMW as the robot? Same network? Discovery server set if the robot uses one?')
        else:
            await add('dds', 'ROS 2 discovery', 'ok', f"{len(nodes)} nodes, {pr['n_topics']} topics visible")

        hz, frame = pr['scan_hz'], pr['scan_frame']
        if hz == 0:
            await add('scan', 'Lidar /scan', 'fail', 'no scan messages in 6 s',
                      'Is the RPLIDAR spinning? Is the robot on and undocked? On the robot: ros2 topic hz /scan.')
        elif pr['scan_valid'] is False:
            await add('scan', 'Lidar /scan', 'fail',
                      f"{hz} Hz but the ranges are constant ({pr['scan_span']} m): the lidar is not seeing the world",
                      'Simulator: set Render to software (the GPU path returns a broken lidar). Real robot: check the lidar is spinning and not covered.')
        else:
            await add('scan', 'Lidar /scan', 'ok' if hz >= 4 else 'warn',
                      f"{hz} Hz, frame '{frame}', publisher {pr['scan_qos']}",
                      '' if hz >= 4 else 'Low rate: Hector will smear the map. Check Wi-Fi quality.')
        d = pr.get('scan_delay')
        if d is not None and not self.sim:
            # Scan stamps come from the robot's clock. Normal is ~0.1-0.3 s (one lidar revolution + Wi-Fi);
            # more means the clocks differ. nav2.yaml is set up not to need synced clocks (collision
            # monitor source_timeout 0, no base shift correction), so this only informs.
            if abs(d) <= 0.5:
                await add('clock', 'Clock sync', 'ok', f'scans arrive {d} s after their timestamp')
            else:
                await add('clock', 'Clock sync', 'warn',
                          f'robot and this computer clocks differ by ~{d} s (Nav2 is configured to cope)',
                          'Optional: sync both with NTP (timedatectl / chrony) for tidier logs and rosbags.')
        if pr['odom_hz'] == 0:
            await add('odom', 'Odometry /odom', 'fail', 'no odometry messages',
                      'The Create 3 is not publishing: check its connection to the Raspberry Pi and the namespace setting.')
        else:
            await add('odom', 'Odometry /odom', 'ok' if pr['odom_hz'] >= 5 else 'warn', f"{pr['odom_hz']} Hz")

        if not pr['odom_base']:
            await add('tf', 'TF odom → base_link', 'fail', 'transform not available',
                      'Create 3 TF is not reaching this computer (check /tf QoS, namespace, discovery).')
        elif not pr['scan_tf']:
            await add('tf', 'TF base_link → scan frame', 'fail', f"no transform to '{frame}'",
                      'Hector needs the lidar frame in TF (robot_state_publisher on the robot).')
        else:
            await add('tf', 'TF tree', 'ok', f"odom → base_link → {frame}")

        ns = self.ns
        mode = s['cmd_vel_mode']
        stamped_ok = pr['cmd_vel_subs'] > 0 and any('TwistStamped' in t for t in pr['cmd_vel_types'])
        plain_ok = pr['unstamped_subs'] > 0
        running_relay = self.procs['relay'].running
        need = self.preflight['stamper_needed'] if running_relay else (mode == 'stamped' or (mode == 'auto' and not plain_ok))
        if mode == 'plain' or (plain_ok and not running_relay):
            await add('cmd', 'Velocity commands', 'ok', f'{ns}/cmd_vel_unstamped (Twist) is accepted')
        elif need and (stamped_ok or running_relay or mode == 'stamped'):
            await add('cmd', 'Velocity commands', 'warn', f'robot takes TwistStamped on {ns}/cmd_vel: a relay will be started',
                      'Handled automatically when you press Start.')
        else:
            await add('cmd', 'Velocity commands', 'fail', 'nothing on the robot accepts velocity commands',
                      'Check the Create 3 is connected and that cmd_vel is not remapped.')
        stamper_needed = bool(need)

        if self.procs['slam'].running:
            await add('conflict', 'No competing SLAM/Nav2', 'ok', 'our own stack is running')
        else:
            bad = [n for n in nodes if re.search(r'slam_toolbox|amcl|map_server|bt_navigator|controller_server|hector', n)]
            if bad or pr['map_odom']:
                await add('conflict', 'No competing SLAM/Nav2', 'fail',
                          'found: ' + ', '.join(bad) if bad else 'something already publishes map → odom',
                          'Stop the robot\'s stock slam/localization/nav2 (turtlebot4_navigation) before starting Hector.')
            else:
                await add('conflict', 'No competing SLAM/Nav2', 'ok', 'no map frame, no navigation nodes')

        b, d = pr['battery'], pr['docked']
        if s['target'] == 'webots':
            await add('state', 'Robot state', 'ok', 'simulated robot (no dock in Webots)')
        elif d:
            await add('state', 'Robot state', 'warn', 'robot is docked', 'Undock it first: ros2 action send_goal /undock irobot_create_msgs/action/Undock {}')
        elif b is not None and b < 0.2:
            await add('state', 'Robot state', 'warn', f'battery {round(b * 100)} %', 'Low battery: charge before a long run.')
        elif d is None:
            await add('state', 'Robot state', 'warn', 'dock state unknown' + (f', battery {round(b * 100)} %' if b is not None else ''),
                      'No /dock_status received (sim or older firmware). Make sure the robot is undocked.')
        else:
            await add('state', 'Robot state', 'ok', 'undocked' + (f', battery {round(b * 100)} %' if b is not None else ''))
        await self.finish_preflight(steps, stamper_needed)

    async def finish_preflight(self, steps, stamper_needed=False):
        ok = bool(steps) and not any(x['status'] == 'fail' for x in steps)
        self.preflight = {'steps': steps, 'ok': ok, 'ts': time.time(), 'stamper_needed': stamper_needed}
        await self.broadcast({'type': 'preflight_done', 'ok': ok, 'ts': self.preflight['ts']})
        (self.session_dir / 'preflight.json').write_text(json.dumps(self.preflight, indent=2))

    # ---- run control ----
    async def start_slam(self, explore, force):
        if self.procs['slam'].running:
            return 'already running'
        age = time.time() - self.preflight['ts']
        if not force and (not self.preflight['ok'] or age > 600):
            return 'Run the connection checks first (they must pass and be < 10 min old).'
        s = self.settings
        if self.preflight['stamper_needed'] or s['cmd_vel_mode'] == 'stamped':
            if not self.procs['relay'].running:
                ns = self.ns
                await self.procs['relay'].start(['ros2', 'run', 'tb4_hector_slam', 'cmd_vel_stamper.py',
                                                 '--in', f'{ns}/cmd_vel_unstamped', '--out', f'{ns}/cmd_vel'])
        argv = ['ros2', 'launch', 'tb4_hector_slam', 'slam_nav_launch.py',
                f"use_sim_time:={'true' if self.sim else 'false'}",
                f"explore:={'true' if explore else 'false'}", f"use_rviz:={'true' if self.windows else 'false'}"]
        if self.ns:
            argv.append('namespace:=' + self.ns)
        await self.procs['slam'].start(argv)
        self.procs['helper'].write({'cmd': 'traj_start', 'path': str(self.session_dir / 'trajectory.csv')})
        return 'started'

    async def stop_slam(self):
        self.procs['helper'].write({'cmd': 'estop'})
        self.procs['helper'].write({'cmd': 'traj_stop'})
        await self.procs['slam'].stop()
        await self.procs['relay'].stop()

    async def estop(self):
        self.procs['helper'].write({'cmd': 'estop'})
        await self.broadcast({'type': 'alert', 'text': 'EMERGENCY STOP sent: goals cancelled, zero velocity published. '
                              'The stack is still running; press Stop to shut it down.'})

    # ---- Gazebo simulation (headless server + robot spawn) ----
    async def start_sim(self):
        if not self.sim:
            return 'Switch the target to a simulator first.'
        if self.sim_state != 'stopped':
            return 'simulation already ' + self.sim_state
        if self.settings['target'] == 'webots':
            return await self.start_webots()
        s = self.settings
        world = s['sim_world'] if s['sim_world'] in WORLDS['gazebo'] else 'maze'
        self.sim_state = 'starting'
        await self.send_procs()
        gpu = os.path.exists('/dev/dri') and s['sim_render'] != 'software'
        ready = asyncio.Event()

        def on_line(line):
            if 'Serving world controls' in line:
                ready.set()
            return False

        script = ('export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix turtlebot4_description)/share:'
                  '$(ros2 pkg prefix irobot_create_description)/share:$GZ_SIM_RESOURCE_PATH; '
                  + ('' if gpu else 'export LIBGL_ALWAYS_SOFTWARE=1; ')
                  # a GPU plus a real display renders the sensors on the GPU; otherwise a virtual display is needed
                  # (without any display OGRE aborts)
                  + ('exec ' if gpu and self.gui_available else 'exec xvfb-run -a -s "-screen 0 1280x1024x24" ')
                  + f'gz sim -s -r --headless-rendering -v 3 /opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds/{world}.sdf')
        await self.procs['sim'].start(['bash', '-c', script], on_line=on_line)
        try:
            await asyncio.wait_for(ready.wait(), 90)
        except asyncio.TimeoutError:
            await self.stop_sim()
            return 'Gazebo server did not start in 90 s: see the Simulation log.'
        await asyncio.sleep(3)
        # robot spawn + bridges; the clock bridge is started by turtlebot4_gz.launch.py normally, not by spawn
        await self.procs['sim_robot'].start(['bash', '-c',
            "ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' & "
            f"exec ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py world:={world} "
            f"model:={s['sim_model']} rviz:=false"])
        self.sim_state = 'ready'
        await self.send_procs()
        await self.start_helper()
        return 'started' + ('' if gpu else ' (software rendering: expect a slow simulation)')

    async def start_webots(self):
        """Webots (installed in the image) + the ROS 2 bridge extern controller."""
        s = self.settings
        self.sim_state = 'starting'
        await self.send_procs()
        ready = asyncio.Event()

        def on_line(line):
            if 'Waiting for local or remote connection' in line:
                ready.set()
            return False

        world = f"/ws/webots/worlds/{s['sim_world']}.wbt"
        if self.windows:
            argv = ['webots', '--stdout', '--stderr', world]
        else:
            argv = ['xvfb-run', '-a', 'webots', '--mode=realtime', '--no-rendering', '--minimize', '--batch',
                    '--stdout', '--stderr', world]
        await self.procs['sim'].start(argv, on_line=on_line)
        try:
            await asyncio.wait_for(ready.wait(), 120)
        except asyncio.TimeoutError:
            await self.stop_sim()
            return 'Webots did not start in 120 s: see the Simulation log.'
        await asyncio.sleep(1)
        await self.procs['sim_robot'].start(['bash', '-c',
            'export PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH '
            'WEBOTS_CONTROLLER_URL=tcp://localhost:1234/Turtlebot4; '
            'exec python3 /ws/webots/controllers/tb4_ros2_bridge/tb4_ros2_bridge.py --layout layout'])
        await asyncio.sleep(6)
        self.sim_state = 'ready'
        await self.send_procs()
        await self.start_helper()
        return 'started' + ('; Webots window opening' if self.windows else '')

    async def stop_sim(self):
        if self.procs['slam'].running:
            await self.stop_slam()
        await self.procs['sim_gui'].stop(5)
        await self.procs['sim_robot'].stop(10)
        await self.procs['sim'].stop(10)
        self.sim_state = 'stopped'
        await self.send_procs()

    # ---- evidence ----
    async def save_map(self, name):
        name = re.sub(r'[^A-Za-z0-9_-]', '', name) or 'map'
        ns = self.ns
        p = self.procs['map_saver']
        await p.start(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(self.session_dir / name),
                       '-t', f'{ns}/map', '--ros-args', '-p', f"use_sim_time:={'true' if self.sim else 'false'}"])
        try:
            await asyncio.wait_for(p.p.wait(), 30)
        except asyncio.TimeoutError:
            await p.stop(2)
        return (self.session_dir / f'{name}.yaml').exists()

    def map_stats(self, yaml_path):
        meta = {}
        for line in yaml_path.read_text().splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
        img = yaml_path.parent / meta['image']
        with open(img, 'rb') as f:
            f.readline()
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            w, h = map(int, line.split())
            f.readline()
            px = f.read(w * h)
        res = float(meta['resolution'])
        occ_t, free_t = float(meta['occupied_thresh']), float(meta['free_thresh'])
        occ = free = 0
        for v in px:
            p = (255 - v) / 255 if meta.get('negate', '0') == '0' else v / 255
            occ += p > occ_t
            free += p < free_t
        return {'map': yaml_path.name, 'resolution_m': res, 'width_cells': w, 'height_cells': h,
                'occupied_cells': occ, 'free_cells': free, 'explored_area_m2': round((occ + free) * res * res, 2)}

    def export(self):
        d = self.session_dir
        summary = {'session': self.session, 'settings': self.settings, 'preflight': self.preflight,
                   'last_status': self.cache.get('status'), 'maps': []}
        for y in sorted(d.glob('*.yaml')):
            try:
                summary['maps'].append(self.map_stats(y))
            except Exception as e:
                summary['maps'].append({'map': y.name, 'error': repr(e)})
        (d / 'summary.json').write_text(json.dumps(summary, indent=2, default=str))
        for n in ('slam', 'relay', 'helper'):
            (d / f'log_{n}.txt').write_text('\n'.join(self.procs[n].log))
        zpath = d.with_suffix('.zip')
        with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
            for f in d.rglob('*'):
                if f.is_file():
                    z.write(f, f.relative_to(d.parent))
        return zpath.name


app_state = App()


# ---- HTTP ----
async def index(request):
    return web.FileResponse(APP / 'static' / 'index.html')


async def ws_handler(request):
    ws = web.WebSocketResponse(max_msg_size=2 ** 24, heartbeat=20)
    await ws.prepare(request)
    st = app_state
    st.clients.add(ws)
    await ws.send_str(json.dumps({'type': 'settings', 'settings': st.settings}))
    for m in st.cache.values():
        await ws.send_str(json.dumps(m))
    await ws.send_str(json.dumps({'type': 'preflight_replay', **st.preflight}))
    await ws.send_str(json.dumps(st.procs_msg()))
    for n, p in st.procs.items():
        await ws.send_str(json.dumps({'type': 'log_replay', 'proc': n, 'lines': list(p.log)[-300:]}))
    async for msg in ws:
        if msg.type != WSMsgType.TEXT:
            continue
        c = json.loads(msg.data)
        if c.get('cmd') == 'vel':          # teleop, 10 Hz while a key is held; clamped here
            s = st.settings
            lin = max(-s['max_lin'], min(s['max_lin'], float(c['lin'])))
            ang = max(-s['max_ang'], min(s['max_ang'], float(c['ang'])))
            st.procs['helper'].write({'cmd': 'vel', 'lin': lin, 'ang': ang,
                                      'mode': 'stamped' if s['cmd_vel_mode'] == 'stamped' else 'auto'})
        elif c.get('cmd') in ('goal', 'cancel', 'undock', 'dock', 'lidar_start', 'lidar_stop', 'goto_start'):
            st.procs['helper'].write(c)
        elif c.get('cmd') == 'estop':
            await st.estop()
    st.clients.discard(ws)
    return ws


def jr(fn):
    async def inner(request):
        try:
            return web.json_response(await fn(request))
        except Exception as e:
            return web.json_response({'error': repr(e)}, status=500)
    return inner


@jr
async def api_settings(request):
    new = await request.json()
    s = dict(DEFAULTS)
    for k, default in DEFAULTS.items():
        v = new.get(k, app_state.settings[k])
        s[k] = type(default)(v) if not isinstance(default, bool) else bool(v)
    if s['rmw'] not in RMWS:
        raise ValueError('rmw must be one of ' + ', '.join(RMWS))
    if s['target'] not in ('real', 'gazebo', 'webots'):
        raise ValueError('invalid target')
    if s['target'] != 'real' and s['sim_world'] not in WORLDS[s['target']]:
        s['sim_world'] = WORLDS[s['target']][0]       # target changed: pick that simulator's default world
    if s['sim_world'] not in WORLDS['gazebo'] + WORLDS['webots'] or s['sim_model'] not in ('standard', 'lite') or s['sim_render'] not in ('auto', 'software'):
        raise ValueError('invalid target / world / model')
    if app_state.procs['slam'].running:
        raise ValueError('Stop the SLAM stack before changing settings.')
    if app_state.sim_state != 'stopped' and (s['target'] != app_state.settings['target'] or s['sim_world'] != app_state.settings['sim_world']
                                             or s['sim_model'] != app_state.settings['sim_model'] or s['sim_render'] != app_state.settings['sim_render']
                                             or s['sim_domain_id'] != app_state.settings['sim_domain_id']):
        raise ValueError('Stop the simulation before changing the target, world, model or domain.')
    app_state.settings = s
    SETTINGS_FILE.write_text(json.dumps(s, indent=2))
    app_state.preflight = {'steps': [], 'ok': False, 'ts': 0, 'stamper_needed': False}
    await app_state.broadcast({'type': 'settings', 'settings': s})
    await app_state.broadcast({'type': 'preflight_replay', **app_state.preflight})
    await app_state.start_helper()
    return {'ok': True}


@jr
async def api_preflight(request):
    await app_state.run_preflight()
    return {'ok': app_state.preflight['ok']}


@jr
async def api_start(request):
    b = await request.json()
    return {'result': await app_state.start_slam(bool(b.get('explore')), bool(b.get('force')))}


@jr
async def api_sim(request):
    if request.match_info['action'] == 'start':
        return {'result': await app_state.start_sim()}
    await app_state.stop_sim()
    return {'result': 'stopped'}


@jr
async def api_stop(request):
    await app_state.stop_slam()
    return {'ok': True}


@jr
async def api_estop(request):
    await app_state.estop()
    return {'ok': True}


@jr
async def api_save_map(request):
    b = await request.json()
    return {'ok': await app_state.save_map(b.get('name', 'map')), 'dir': str(app_state.session_dir)}


@jr
async def api_upload(request):
    kind = request.match_info['kind']                  # screenshot | video
    ext = {'screenshot': 'png', 'video': 'webm'}[kind]
    path = app_state.session_dir / f'{kind}_{time.strftime("%H%M%S")}.{ext}'
    path.write_bytes(await request.read())
    return {'saved': path.name}


@jr
async def api_bag(request):
    st = app_state
    action = request.match_info['action']
    p = st.procs['bag']
    if action == 'start' and not p.running:
        ns = st.ns
        topics = [f'{ns}/{t}' for t in ('scan', 'odom', 'map', 'tf', 'tf_static')]
        await p.start(['ros2', 'bag', 'record', '-o', str(st.session_dir / 'rosbag'), *topics])
    elif action == 'stop':
        await p.stop()
    return {'running': p.running}


@jr
async def api_export(request):
    app_state.procs['helper'].write({'cmd': 'traj_stop'})
    await asyncio.sleep(0.3)
    return {'zip': app_state.export()}


@jr
async def api_new_session(request):
    app_state.session = time.strftime('%Y%m%d_%H%M%S')
    await app_state.send_procs()
    return {'session': app_state.session}


async def on_start(app):
    await app_state.start_helper()


async def on_stop(app):
    for p in app_state.procs.values():
        await p.stop(5)


def main():
    app = web.Application(client_max_size=2 ** 28)
    app.add_routes([
        web.get('/', index), web.get('/ws', ws_handler),
        web.post('/api/settings', api_settings), web.post('/api/preflight', api_preflight),
        web.post('/api/start', api_start), web.post('/api/stop', api_stop), web.post('/api/sim/{action}', api_sim), web.post('/api/estop', api_estop),
        web.post('/api/save_map', api_save_map), web.post('/api/upload/{kind}', api_upload),
        web.post('/api/bag/{action}', api_bag), web.post('/api/export', api_export),
        web.post('/api/new_session', api_new_session),
        web.static('/results', DATA / 'results'),
    ])
    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)
    web.run_app(app, host='0.0.0.0', port=int(os.environ.get('PORT', 8080)), print=lambda *_: None)


if __name__ == '__main__':
    print(f"Dashboard on http://localhost:{os.environ.get('PORT', 8080)}", flush=True)
    main()
