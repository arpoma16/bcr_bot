"""Import the action orchestration offline, without ROS or an active graph."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Goal(Message):
    def __init__(self, **kwargs):
        self.target = SimpleNamespace(x=0.0)
        super().__init__(**kwargs)


class Result(Message):
    DOCK_NOT_IN_DB = 901
    DOCK_NOT_VALID = 902
    FAILED_TO_STAGE = 903
    FAILED_TO_DETECT_DOCK = 904
    FAILED_TO_CONTROL = 905


class Feedback(Message):
    NAV_TO_STAGING_POSE = 1
    INITIAL_PERCEPTION = 2
    CONTROLLING = 3


@pytest.fixture
def handler_module(monkeypatch):
    modules = {
        'rclpy': [], 'builtin_interfaces.msg': ['Duration'],
        'geometry_msgs.msg': ['PoseStamped', 'PolygonStamped', 'Point32', 'Polygon'],
        'nav2_msgs.action': ['BackUp', 'DockRobot', 'DriveOnHeading', 'NavigateToPose',
                             'Spin', 'UndockRobot'],
        'rclpy.action': ['ActionClient', 'ActionServer', 'CancelResponse', 'GoalResponse'],
        'rclpy.callback_groups': ['ReentrantCallbackGroup'],
        'rclpy.duration': ['Duration'], 'rclpy.executors': ['MultiThreadedExecutor'],
        'rclpy.node': ['Node'],
        'rclpy.qos': ['DurabilityPolicy', 'QoSProfile', 'ReliabilityPolicy',
                      'qos_profile_sensor_data'],
        'rclpy.time': ['Time'], 'sensor_msgs.msg': ['LaserScan'], 'std_msgs.msg': ['Bool'],
        'std_srvs.srv': ['Empty', 'Trigger', 'SetBool'],
        'tf2_ros': ['Buffer', 'TransformException', 'TransformListener'],
    }
    for name, names in modules.items():
        module = ModuleType(name)
        for attr in names:
            value = type(attr, (Message,), {})
            if name == 'nav2_msgs.action':
                value.Goal, value.Result, value.Feedback = Goal, Result, Feedback
            if name == 'std_srvs.srv':
                value.Request = Message
            if attr == 'TransformException':
                value = type(attr, (Exception,), {})
            setattr(module, attr, value)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('offline_cart_handler', SCRIPTS / 'cart_handler.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def adapter_module(handler_module):
    spec = importlib.util.spec_from_file_location('offline_cart_adapter', SCRIPTS / 'cart_carry_adapter.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
