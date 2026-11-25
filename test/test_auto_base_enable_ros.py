"""Synthetic owner on isolated ROS domain 228; no SDK or robot connection."""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))


def test_python_dispatch_recovers_repeated_holds_with_only_zero_commands():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Twist
    from local_planner_node import LocalPlanner
    from test_auto_base_enable import owner

    context = Context()
    rclpy.init(context=context, domain_id=228)
    executor = SingleThreadedExecutor(context=context)
    planner = fake = None
    try:
        planner = LocalPlanner(context=context, parameter_overrides=[
            Parameter('dispatch', value=True), Parameter('ignore_lidar', value=True)])
        fake = rclpy.create_node('synthetic_base_enable_owner', context=context)
        state, commands, calls = owner(), [], []
        status = fake.create_publisher(String, '/astribot/base/status', 1)
        def command(msg):
            commands.append((msg.linear.x, msg.linear.y, msg.angular.z))
        fake.create_subscription(Twist, '/cmd_vel', command, 1)
        fake.create_timer(.05, lambda: status.publish(String(data=json.dumps(state))))
        def enable(request, response):
            calls.append(request.data)
            assert commands and all(v == (0., 0., 0.) for v in commands)
            state.update(mode='MODEL', armed=True, reason='command_accepted', generation=state['generation']+1)
            response.success, response.message = True, 'enabled'
            return response
        fake.create_service(SetBool, '/astribot/base/enable', enable)
        executor.add_node(planner)
        executor.add_node(fake)
        end = time.monotonic() + 5.
        while time.monotonic() < end and not calls:
            executor.spin_once(timeout_sec=.02)
        assert calls == [True]
        assert not planner.auto_base_enable.done
        state.update(mode='HOLD', armed=False, reason='publisher_conflict_or_missing', generation=9)
        end = time.monotonic() + 3.
        while time.monotonic() < end and len(calls) < 2:
            executor.spin_once(timeout_sec=.02)
        assert calls == [True, True]
        # Explicit operator disable remains latched even while the planner runs.
        state.update(mode='HOLD', armed=False, reason='disabled', generation=11)
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        assert calls == [True, True]
        assert commands and all(v == (0., 0., 0.) for v in commands)
    finally:
        executor.shutdown()
        if planner is not None:
            planner.destroy_node()
        if fake is not None:
            fake.destroy_node()
        context.shutdown()
