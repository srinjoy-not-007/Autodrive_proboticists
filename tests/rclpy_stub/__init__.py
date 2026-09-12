"""A lightweight stand-in for the parts of rclpy and the ROS message packages
that `pit_crew`'s nodes touch, so the real node classes can be CONSTRUCTED and
DRIVEN on the host.

Why this exists: rclpy and the message packages are not installed on the dev
host, so every host test so far has either lifted methods out with `ast` or
tested the pure-python cores.  A refactor renamed `_speed_cap` -> `speed_cap`
in `localization.py`, left the old call in `__init__`, and 200 host tests plus
`py_compile` all passed while the node died on the first line of the race.

`install()` inserts these modules into `sys.modules` BEFORE `pit_crew.*` is
imported.  Nothing here tries to be ROS: publishers record, subscriptions
record, timers record, the logger is a sink, and the clock is `time.time()`
plus a test-controlled offset so a 30-cycle run at 0.1 s spacing can be driven
in milliseconds of real time.
"""

import sys
import time
import types

# --------------------------------------------------------------------- clock

_OFFSET = 0.0


def now_s():
    """The stub clock, in seconds: real time plus the test's offset."""
    return time.time() + _OFFSET


def advance(dt):
    """Move the stub clock forward by `dt` seconds of virtual time."""
    global _OFFSET
    _OFFSET += float(dt)
    return now_s()


def reset_clock():
    global _OFFSET
    _OFFSET = 0.0


# ----------------------------------------------------------------- messages

class _Msg:
    """Fields come from `_defaults()`; keywords override, as in rosidl."""

    def __init__(self, **kw):
        for name, make in self._defaults().items():
            setattr(self, name, make() if callable(make) else make)
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError(f'{type(self).__name__} has no field {k}')
            setattr(self, k, v)

    @staticmethod
    def _defaults():
        return {}

    def __repr__(self):
        return f'{type(self).__name__}({self.__dict__})'


class Time(_Msg):
    @staticmethod
    def _defaults():
        return {'sec': 0, 'nanosec': 0}


class Header(_Msg):
    @staticmethod
    def _defaults():
        return {'stamp': Time, 'frame_id': ''}


class Vector3(_Msg):
    @staticmethod
    def _defaults():
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}


Point = type('Point', (Vector3,), {})


class Quaternion(_Msg):
    @staticmethod
    def _defaults():
        return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


class Pose(_Msg):
    @staticmethod
    def _defaults():
        return {'position': Point, 'orientation': Quaternion}


class PoseStamped(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'pose': Pose}


class PoseArray(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'poses': list}


class PoseWithCovariance(_Msg):
    @staticmethod
    def _defaults():
        return {'pose': Pose, 'covariance': lambda: [0.0] * 36}


class Twist(_Msg):
    @staticmethod
    def _defaults():
        return {'linear': Vector3, 'angular': Vector3}


class TwistWithCovariance(_Msg):
    @staticmethod
    def _defaults():
        return {'twist': Twist, 'covariance': lambda: [0.0] * 36}


class Imu(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'orientation': Quaternion,
                'orientation_covariance': lambda: [0.0] * 9,
                'angular_velocity': Vector3,
                'angular_velocity_covariance': lambda: [0.0] * 9,
                'linear_acceleration': Vector3,
                'linear_acceleration_covariance': lambda: [0.0] * 9}


class JointState(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'name': list, 'position': list,
                'velocity': list, 'effort': list}


class LaserScan(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'angle_min': 0.0, 'angle_max': 0.0,
                'angle_increment': 0.0, 'time_increment': 0.0,
                'scan_time': 0.0, 'range_min': 0.0, 'range_max': 0.0,
                'ranges': list, 'intensities': list}


class Odometry(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'child_frame_id': '',
                'pose': PoseWithCovariance, 'twist': TwistWithCovariance}


class Path(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'poses': list}


class OccupancyGrid(_Msg):
    @staticmethod
    def _defaults():
        return {'header': Header, 'info': None, 'data': list}


def _scalar(name, zero):
    return type(name, (_Msg,), {'_defaults': staticmethod(lambda: {'data': zero})})


Float32 = _scalar('Float32', 0.0)
Float64 = _scalar('Float64', 0.0)
Int32 = _scalar('Int32', 0)
Bool = _scalar('Bool', False)
String = _scalar('String', '')


# -------------------------------------------------------------------- rclpy

class Parameter:
    """Only what `[x.value for x in p]` and `{n.name: n.value}` need."""

    def __init__(self, name, value):
        self.name = name
        self._value = value

    @property
    def value(self):
        return self._value

    def get_parameter_value(self):
        return self

    def __repr__(self):
        return f'Parameter({self.name!r}, {self._value!r})'


class _Logger:
    """A no-op logger that keeps what it was told, so a test can assert that a
    node did not log an error it would otherwise swallow (`mpc3.control_step`
    catches every exception by design)."""

    def __init__(self):
        self.records = []

    def _at(self, level):
        def log(msg, *_a, **_kw):
            self.records.append((level, str(msg)))
        return log

    def __getattr__(self, name):
        if name in ('debug', 'info', 'warn', 'warning', 'error', 'fatal'):
            return self._at('warn' if name == 'warning' else name)
        raise AttributeError(name)

    def errors(self):
        return [m for lvl, m in self.records if lvl in ('error', 'fatal')]


class _Clock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=int(now_s() * 1e9))


class Publisher:
    def __init__(self, msg_type, topic):
        self.msg_type = msg_type
        self.topic = topic
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    def get_subscription_count(self):
        return 0

    def destroy(self):
        pass


class Subscription:
    def __init__(self, msg_type, topic, callback):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback


class Timer:
    def __init__(self, period, callback):
        self.timer_period_s = float(period)
        self.callback = callback
        self.calls = 0

    def fire(self):
        self.calls += 1
        return self.callback()

    def cancel(self):
        pass


_GLOBAL_OVERRIDES = {}


def set_overrides(values=None, **kw):
    """The parameters a launch would pass with `-p name:=value`.

    The nodes call `super().__init__('name')` with no `parameter_overrides`, so
    a test cannot hand them in through the constructor; this is the seam.
    """
    global _GLOBAL_OVERRIDES
    _GLOBAL_OVERRIDES = dict(values or {})
    _GLOBAL_OVERRIDES.update(kw)
    return _GLOBAL_OVERRIDES


class Node:
    """Records everything a node asks the middleware for.

    `parameter_overrides` is how a test supplies the `-p name:=value` a launch
    would: `declare_parameters` returns the override where one is given and the
    declared default otherwise, which is exactly what rclpy does.
    """

    def __init__(self, node_name, parameter_overrides=None, **_kw):
        self._stub_name = node_name
        self._stub_overrides = dict(_GLOBAL_OVERRIDES)
        self._stub_overrides.update(parameter_overrides or {})
        self._stub_params = {}
        self._stub_subs = []
        self._stub_pubs = {}
        self._stub_timers = []
        self._stub_logger = _Logger()
        self._stub_clock = _Clock()

    # -- parameters
    def declare_parameters(self, namespace, parameters):
        out = []
        for name, default in parameters:
            full = f'{namespace}.{name}' if namespace else name
            value = self._stub_overrides.get(name,
                                             self._stub_overrides.get(full,
                                                                      default))
            self._stub_params[name] = value
            out.append(Parameter(full, value))
        return out

    def declare_parameter(self, name, default=None, *_a, **_kw):
        return self.declare_parameters('', [(name, default)])[0]

    def get_parameter(self, name):
        return Parameter(name, self._stub_params[name])

    def set_parameters(self, params):
        for p in params:
            self._stub_params[p.name] = p.value

    # -- pub / sub / timers
    def create_subscription(self, msg_type, topic, callback, qos_profile,
                            **_kw):
        sub = Subscription(msg_type, topic, callback)
        self._stub_subs.append(sub)
        return sub

    def create_publisher(self, msg_type, topic, qos_profile, **_kw):
        pub = Publisher(msg_type, topic)
        self._stub_pubs[topic] = pub
        return pub

    def create_timer(self, period, callback, **_kw):
        t = Timer(period, callback)
        self._stub_timers.append(t)
        return t

    # -- misc node API
    def get_logger(self):
        return self._stub_logger

    def get_clock(self):
        return self._stub_clock

    def get_name(self):
        return self._stub_name

    def destroy_node(self):
        pass

    # -- test helpers
    def stub_deliver(self, topic, msg):
        """Hand `msg` to every subscription whose topic ends with `topic`."""
        hits = [s for s in self._stub_subs
                if s.topic == topic or s.topic.endswith(topic)]
        if not hits:
            raise AssertionError(
                f'nothing subscribed to {topic!r}; have '
                f'{[s.topic for s in self._stub_subs]}')
        for s in hits:
            s.callback(msg)
        return len(hits)

    def stub_fire_timers(self):
        for t in list(self._stub_timers):
            t.fire()

    def stub_published(self, topic):
        for name, pub in self._stub_pubs.items():
            if name == topic or name.endswith(topic):
                return pub.msgs
        raise AssertionError(
            f'nothing published on {topic!r}; have {list(self._stub_pubs)}')


# ------------------------------------------------------------------ install

_INITIALISED = False


def init(*_a, **_kw):
    global _INITIALISED
    _INITIALISED = True


def shutdown(*_a, **_kw):
    global _INITIALISED
    _INITIALISED = False


def try_shutdown(*_a, **_kw):
    shutdown()


def ok(*_a, **_kw):
    return _INITIALISED


def spin(_node, *_a, **_kw):
    raise RuntimeError('rclpy_stub.spin: nothing to spin on the host')


def spin_once(_node, *_a, **_kw):
    return None


def _module(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    return mod


def install():
    """Insert the stub modules into `sys.modules`.  Idempotent."""
    if 'rclpy' in sys.modules and getattr(sys.modules['rclpy'],
                                          '_is_pit_crew_stub', False):
        return sys.modules['rclpy']

    qos = _module(
        'rclpy.qos',
        QoSProfile=type('QoSProfile', (), {
            '__init__': lambda self, depth=10, reliability=None, history=None,
            durability=None, **kw: self.__dict__.update(
                depth=depth, reliability=reliability, history=history,
                durability=durability)}),
        QoSReliabilityPolicy=types.SimpleNamespace(
            BEST_EFFORT=1, RELIABLE=2, SYSTEM_DEFAULT=0),
        QoSHistoryPolicy=types.SimpleNamespace(
            KEEP_LAST=1, KEEP_ALL=2, SYSTEM_DEFAULT=0),
        QoSDurabilityPolicy=types.SimpleNamespace(
            VOLATILE=1, TRANSIENT_LOCAL=2, SYSTEM_DEFAULT=0),
    )
    node_mod = _module('rclpy.node', Node=Node, Parameter=Parameter)
    param_mod = _module('rclpy.parameter', Parameter=Parameter)
    dur_mod = _module('rclpy.duration', Duration=type(
        'Duration', (), {'__init__': lambda self, seconds=0.0, nanoseconds=0:
                         self.__dict__.update(
                             nanoseconds=int(seconds * 1e9) + nanoseconds)}))
    rclpy = _module('rclpy', init=init, shutdown=shutdown,
                    try_shutdown=try_shutdown, ok=ok, spin=spin,
                    spin_once=spin_once, node=node_mod, qos=qos,
                    parameter=param_mod, duration=dur_mod,
                    Parameter=Parameter, _is_pit_crew_stub=True)

    def msg_pkg(pkg, **classes):
        msg = _module(f'{pkg}.msg', **classes)
        top = _module(pkg, msg=msg)
        sys.modules[pkg] = top
        sys.modules[f'{pkg}.msg'] = msg

    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod
    sys.modules['rclpy.qos'] = qos
    sys.modules['rclpy.parameter'] = param_mod
    sys.modules['rclpy.duration'] = dur_mod

    msg_pkg('sensor_msgs', Imu=Imu, JointState=JointState, LaserScan=LaserScan)
    msg_pkg('geometry_msgs', PoseStamped=PoseStamped, PoseArray=PoseArray,
            Pose=Pose, Point=Point, Quaternion=Quaternion, Vector3=Vector3,
            Twist=Twist, PoseWithCovariance=PoseWithCovariance,
            TwistWithCovariance=TwistWithCovariance)
    msg_pkg('std_msgs', Float32=Float32, Float64=Float64, Int32=Int32,
            Bool=Bool, String=String, Header=Header)
    msg_pkg('nav_msgs', Odometry=Odometry, Path=Path,
            OccupancyGrid=OccupancyGrid)
    return rclpy
