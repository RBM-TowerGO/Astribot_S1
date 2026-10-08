#!/usr/bin/env python3
"""Standalone S1 terminal teleop. Importing this module never loads the SDK."""
import argparse
import fcntl
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time
import tty

HZ = 100.0
LEASE = 0.2
MAX_CYCLE = 0.1


def speed(value, low, high):
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise argparse.ArgumentTypeError('速度必须在 %g–%g 之间' % (low, high))
    return value


def position(value):
    if len(value) != 3 or not all(math.isfinite(float(x)) for x in value):
        raise RuntimeError('底盘位置必须是有限的 [x, y, yaw]')
    return [float(x) for x in value]


class Keys:
    def __init__(self, linear=.05, angular=.10):
        self.linear, self.angular = linear, angular
        self.key, self.until, self.quit = '', 0., False
        self.message = '等待新按键'

    def stop(self, message):
        self.key, self.until, self.message = '', 0., message

    def feed(self, data, now):
        keys = data.lower()
        if any(k in keys for k in ('x', '\x03', '\x04')):
            self.quit = True
            self.stop('退出')
            return
        if not keys or any(k not in 'wsqe+-=' for k in keys):
            self.stop('左右平移尚未启用' if 'a' in keys or 'd' in keys else '保持目标')
            return
        # Speed changes do not renew the movement lease.
        for k in keys:
            if k in '+-=':
                sign = -1 if k == '-' else 1
                self.linear = round(max(.01, min(.15, self.linear + sign * .01)), 5)
                self.angular = round(max(.025, min(.25, self.angular + sign * .025)), 5)
            else:
                self.key, self.until = k, now + LEASE
                self.message = {'w': '前进', 's': '后退', 'q': '左转', 'e': '右转'}[k]

    def sample(self, now):
        if self.key and now >= self.until:
            self.stop('输入超时：保持目标')
        return {'w': (self.linear, 0.), 's': (-self.linear, 0.),
                'q': (0., self.angular), 'e': (0., -self.angular)}.get(self.key, (0., 0.))


class Driver:
    def __init__(self, backend, now):
        self.backend = backend
        self.target = position(backend.desired())
        self.last = now
        self.failed = False
        self.sent = False

    def tick(self, now, velocity):
        if self.failed:
            raise RuntimeError('控制已故障，不能自动恢复')
        try:
            if not 0 < now - self.last <= MAX_CYCLE:
                raise RuntimeError('控制循环停顿超过 0.1 秒；退出，不补发动作')
            self.last = now
            if not self.backend.has_control():
                raise RuntimeError('控制权丢失；停止发送')
            self.backend.check()
            v, w = velocity
            if not all(math.isfinite(x) for x in (v, w)) or abs(v) > .15 or abs(w) > .25:
                raise RuntimeError('速度无效或超过限位')
            if v or w:
                # Vendor example 202: these are accumulated LOCAL commands,
                # not a world-frame pose. Example 203 converts world velocities
                # into local velocities before the same integration. Rotating
                # our local W/S again introduces unwanted lateral motion.
                self.target[0] += v / HZ
                # target[1] is unchanged: A/D translation is not implemented.
                self.target[2] += w / HZ
                self.sent = True  # send may raise after handing data to the SDK
            if self.sent:
                self.backend.send(list(self.target))
        except Exception:
            self.failed = True
            raise


def conflicts(proc=Path('/proc'), include_teleop=True):
    """Best-effort process detection; a lock separately guards this tool."""
    found = []
    for p in proc.iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            argv = (p / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        except (OSError, ProcessLookupError):
            continue
        names = [Path(a).name for a in argv if a]
        direct = bool(set(names) & {'tg-chassis.py', '202-chassis_joy_control_local.py',
                                    '203-chassis_joy_control_global.py'})
        direct = direct or any(a.endswith('/towergo_real/chassis') for a in argv)
        teleop = include_teleop and 'tg_teleop.py' in names and '--dry-run' not in argv
        ros = any(argv[i:i+3] == ['run', 'towergo_real', 'chassis'] for i in range(len(argv)))
        module = any(argv[i:i+2] == ['-m', 'towergo_real.chassis'] for i in range(len(argv)))
        if direct or ros or module or teleop:
            found.append(p.name)
    return found


class SessionLock:
    def __init__(self):
        # Share the existing ROS chassis backend's lock name.
        base = Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp')
        self.path = base / ('towergo-chassis-%d.lock' % os.getuid())
        self.file = None

    def __enter__(self):
        self.file = self.path.open('a')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.file.close()
            self.file = None
            raise RuntimeError('已有底盘控制实例持有锁，拒绝启动') from None
        return self

    def __exit__(self, *args):
        if self.file:
            self.file.close()


def configure_terminal(fd):
    tty.setcbreak(fd)
    attrs = termios.tcgetattr(fd)
    # cbreak retains XON/XOFF: Ctrl+S can otherwise freeze status output and
    # block the control loop. Treat these bytes as unknown stop keys instead.
    attrs[0] &= ~(termios.IXON | termios.IXOFF | getattr(termios, 'IXANY', 0))
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


class FakeBackend:
    def __init__(self):
        self.p = [0., 0., 0.]

    def desired(self):
        return list(self.p)

    def has_control(self):
        return True

    def check(self):
        pass

    def send(self, p):
        self.p = list(p)

    def close(self):
        pass


class SDKBackend:
    def __init__(self):
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        import astribot_ros_middleware as middleware
        self.bot, self.thread = None, None
        self.done = threading.Event()
        self.error = None
        try:
            self.bot = Astribot(freq=HZ, high_control_rights=True)
            if not self.has_control():
                raise RuntimeError('未获得底盘控制权')
            def spin():
                try:
                    while not self.done.is_set():
                        middleware.spin()
                except Exception as exc:
                    self.error = exc
            self.thread = threading.Thread(target=spin, daemon=True)
            self.thread.start()
            position(self.bot.get_current_joints_position([self.bot.chassis_name])[0])
            position(self.desired())
        except BaseException:
            self.close()
            raise

    def desired(self):
        return self.bot.get_desired_joints_position([self.bot.chassis_name])[0]

    def has_control(self):
        return bool(self.bot and self.bot.get_control_rights_status())

    def check(self):
        if self.error is not None:
            raise RuntimeError('SDK 通信线程异常: %s' % self.error)

    def send(self, p):
        self.bot.set_joints_position([self.bot.chassis_name], [p],
                                    control_way='filter', use_wbc=False, add_default_torso=False)

    def close(self):
        try:
            if self.bot:
                iface = self.bot.astribot_interface
                iface.shutdown()
                end = time.monotonic() + 1.5
                while getattr(iface, 'have_control_rights', False) and time.monotonic() < end:
                    time.sleep(.05)
                if getattr(iface, 'have_control_rights', False):
                    raise RuntimeError('SDK 控制权未在 1.5 秒内释放')
        finally:
            self.done.set()
            if self.thread:
                self.thread.join(timeout=.2)


def finish(driver, backend, dry, clock=time.monotonic, sleep=time.sleep):
    """Best-effort target hold, always followed by release; never integrate."""
    error = None
    try:
        if driver and driver.sent and not dry:
            end = clock() + 3.
            while clock() < end:
                if not backend.has_control():
                    raise RuntimeError('控制权已丢失，未继续保持目标')
                backend.check()
                backend.send(list(driver.target))
                sleep(1. / HZ)
    except Exception as exc:
        error = exc
    finally:
        try:
            backend.close()
        except Exception as exc:
            error = RuntimeError('%s; 释放失败: %s' % (error or '收尾', exc))
    if error:
        raise RuntimeError(str(error))


def run(args):
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        raise RuntimeError('需要交互终端，请使用 ssh -t S1；不接受管道输入')
    keys = Keys(args.linear_speed, args.angular_speed)
    backend = driver = None
    saved = termios.tcgetattr(fd)
    handlers = {}
    signaled = []
    def halt(sig, frame):
        signaled.append(sig)
        keys.quit = True
        keys.stop('收到退出信号')
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, halt)
        with SessionLock():
            try:
                others = conflicts(include_teleop=not args.dry_run)
                if others:
                    raise RuntimeError('检测到其他底盘控制进程 PID=%s，请先退出该程序' % ','.join(others))
                print('演练模式：不连接 SDK' if args.dry_run else
                      '将申请 S1 控制权。软件停止为保持目标，可能存在收敛尾程。', flush=True)
                backend = FakeBackend() if args.dry_run else SDKBackend()
                if keys.quit:
                    return 128 + signaled[0] if signaled else 0
                driver = Driver(backend, time.monotonic())
                configure_terminal(fd)
                print('READY | W/S 前后 · Q/E 左右转 · A/D 预留 · 空格保持 · +/- 调速 · X 退出', flush=True)
                print('坐标模式 SDK202_LOCAL：W/S 仅增减局部 x，Q/E 仅增减 yaw；不作全局旋转。', flush=True)
                print('按键有效期 0.2 秒；首次长按可能短暂停顿。', flush=True)
                show = 0.
                while not keys.quit:
                    # Always wait one fresh period: never catch up delayed cycles.
                    time.sleep(1. / HZ)
                    now = time.monotonic()
                    if select.select([fd], [], [], 0)[0]:
                        data = os.read(fd, 4096)
                        if not data:
                            keys.quit = True
                            break
                        keys.feed(data.decode('ascii', errors='replace'), now)
                    if keys.quit:
                        break
                    driver.tick(now, keys.sample(now))
                    if now >= show:
                        print('\r\033[2K%s | v=%.3f m/s w=%.3f rad/s | local_cmd=[%.4f, %.4f, %.4f]' %
                              (keys.message, keys.linear, keys.angular, *driver.target), end='', flush=True)
                        show = now + .2
                print('\n退出：停止积分，清理控制会话。', flush=True)
                return 128 + signaled[0] if signaled else 0
            finally:
                if backend:
                    finish(driver, backend, args.dry_run)
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSANOW, saved)
        except (OSError, termios.error):
            pass  # SSH may already have closed the terminal.
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description='S1 SSH 底盘遥操；X 退出，Q/E 旋转')
    parser.add_argument('--linear-speed', type=lambda x: speed(x, .01, .15), default=.05)
    parser.add_argument('--angular-speed', type=lambda x: speed(x, .025, .25), default=.10)
    parser.add_argument('--dry-run', action='store_true', help='不加载 SDK，仅演练键盘与目标积分')
    args = parser.parse_args()
    try:
        return run(args)
    except Exception as exc:
        print('\nERROR: %s' % exc, file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
