"""Basic Twist transport with synthetic endpoints on isolated localhost domains."""
import json
import os
from pathlib import Path
import sys
import time

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from orin_basic_bridge import BasicBridge, REMOTE
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool


def main():
    bridge = BasicBridge(211, 212)
    source, output = bridge.source, bridge.output
    owner = output.create_publisher(String, '/astribot/base/status', 1)
    status = source.create_publisher(String, REMOTE+'/status', 1)
    command_pub = source.create_publisher(Twist, REMOTE+'/cmd_vel', 1)
    goal_pub = output.create_publisher(PoseStamped, '/astribot_viz/goal_pose', 1)
    cancel_pub = output.create_publisher(Empty, '/astribot_viz/goal_cancel', 1)
    received, goals, cancels, enabled = [], [], [], []
    output.create_subscription(Twist, '/cmd_vel', received.append, 1)
    source.create_subscription(PoseStamped, REMOTE+'/goal_pose', goals.append, 1)
    source.create_subscription(Empty, REMOTE+'/goal_cancel', cancels.append, 1)
    state = dict(mode='HOLD', armed=False, selected_source='auto', selected_topic='/cmd_vel',
        generation=2, reason='disabled', hardware_writes=True, feedback_source='hardware',
        feedback_age_ms=1., stop_pending=False, stop_confirmed=True,
        command_age_ms=None, requested_ros_velocity=None)

    def enable(request, response):
        assert request.data and received and received[-1].linear.x == 0.
        enabled.append(True)
        state.update(mode='MODEL', armed=True, generation=state['generation']+1, reason='enabled')
        response.success, response.message = True, 'enabled'
        return response

    output.create_service(SetBool, '/astribot/base/enable', enable)
    command, stream, dispatch, active = Twist(), True, False, False
    next_publish = 0.

    def pump(seconds):
        nonlocal next_publish
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                owner.publish(String(data=json.dumps(state)))
                status.publish(String(data=json.dumps(dict(active=active, dispatch=dispatch))))
                if stream:
                    command_pub.publish(command)
                next_publish = time.monotonic()+.05
            bridge.spin_once()

    try:
        command.linear.x = .02
        pump(1.)
        assert not enabled and not any(m.linear.x for m in received)
        command.linear.x = 0.
        pump(.3)
        assert received and not enabled
        state.update(command_age_ms=10., requested_ros_velocity=[0., 0., 0.])
        dispatch = True
        pump(.8)
        assert len(enabled) == 1 and state['armed']
        goal = PoseStamped()
        goal.header.frame_id = 'map'; goal.header.stamp = output.get_clock().now().to_msg()
        goal.pose.position.x = .2; goal.pose.orientation.w = 1.
        goal_pub.publish(goal); pump(.1)
        assert goals and goals[-1].pose.position.x == .2
        active = True; command.linear.x = .02
        pump(.2)
        assert any(m.linear.x == .02 for m in received)
        cancel_pub.publish(Empty()); command.linear.x = 0.; active = False
        pump(.2)
        assert cancels and received[-1].linear.x == 0.
        stream = False; pump(.2); received.clear(); pump(.3)
        assert not received, 'Relay fabricated a command heartbeat'
        source.destroy_publisher(command_pub)
        pump(.2)
        command_pub = source.create_publisher(Twist, REMOTE+'/cmd_vel', 1)
        state.update(mode='HOLD', armed=False, reason='command_timeout', generation=4)
        stream = True; pump(1.2)
        assert len(enabled) == 2 and state['armed'], 'Planner restart needed a manual re-enable'
        duplicate = source.create_publisher(Twist, REMOTE+'/cmd_vel', 1)
        command.linear.x = .02; pump(.2); received.clear(); pump(.2)
        assert not received, 'Conflicting command publishers were forwarded'
        source.destroy_publisher(duplicate)
        command.linear.x = 0.
        state.update(mode='HOLD', armed=False, reason='disabled', generation=6)
        stream = True; pump(.3)
        assert len(enabled) == 2 and bridge.auto_enable.done
        command.linear.x = .02; received.clear(); pump(.2)
        assert not any(m.linear.x for m in received), 'Disabled owner received nonzero command'
        print('PASS: plain Twist, startup enable after zero, PoseStamped goal, cancel, '
              'no synthetic heartbeat, planner restart recovery, conflict blocked, explicit disable preserved')
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
