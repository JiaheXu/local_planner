"""Existing leased relay and auto-enable RPC with a fake owner, localhost only."""
import json
import os
from pathlib import Path
import sys
import time

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'python'))
from orin_dispatch_bridge import DispatchBridge
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool
from planner_transport import REMOTE


def main():
    bridge = DispatchBridge(211, 212)
    source, output = bridge.source, bridge.output
    owner = output.create_publisher(String, '/astribot/base/status', 1)
    status = source.create_publisher(String, REMOTE+'/status', 1)
    commands = source.create_publisher(String, REMOTE+'/command', 1)
    goals = output.create_publisher(PoseStamped, '/astribot_viz/goal_pose', 1)
    cancels = output.create_publisher(Empty, '/astribot_viz/goal_cancel', 1)
    leases, received, remote_goals, enabled = [], [], [], []
    source.create_subscription(String, REMOTE+'/owner_status', lambda m: leases.append(json.loads(m.data)), 1)
    source.create_subscription(String, REMOTE+'/goal', lambda m: remote_goals.append(json.loads(m.data)), 1)
    output.create_subscription(Twist, '/cmd_vel', received.append, 1)
    state = dict(mode='HOLD', armed=False, selected_source='auto', selected_topic='/cmd_vel',
        generation=2, reason='publisher_conflict_or_missing', hardware_writes=True,
        feedback_source='hardware', feedback_age_ms=1., stop_pending=False,
        stop_confirmed=True, command_age_ms=10., requested_ros_velocity=[0.,0.,0.])
    def enable(request, response):
        assert request.data and received and received[-1].linear.x == 0.
        enabled.append(True)
        state.update(mode='MODEL', armed=True, generation=3, reason='enabled')
        response.success, response.message = True, 'enabled'
        return response
    output.create_service(SetBool, '/astribot/base/enable', enable)
    seq, next_publish = 0, 0.
    command, goal_id, dispatch = [0.,0.,0.], None, False
    def pump(seconds):
        nonlocal seq, next_publish
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            if time.monotonic() >= next_publish:
                owner.publish(String(data=json.dumps(state)))
                status.publish(String(data=json.dumps(dict(active=goal_id is not None, dispatch=dispatch))))
                if leases:
                    seq += 1
                    lease = leases[-1]
                    commands.publish(String(data=json.dumps(dict(schema=1, session='synthetic', sequence=seq,
                        lease=lease['transport_lease'], generation=lease['generation'], goal=goal_id, command=command))))
                next_publish = time.monotonic()+.05
            bridge.spin_once()
    try:
        pump(2.)
        assert received and not enabled, 'Shadow mode must not trigger owner enable'
        dispatch = True
        pump(1.)
        assert enabled and state['armed'], 'Dispatch idle did not enable the checked fake owner'
        goal = PoseStamped()
        goal.header.frame_id = 'map'; goal.header.stamp = output.get_clock().now().to_msg()
        goal.pose.position.x = .2; goal.pose.orientation.w = 1.
        goals.publish(goal)
        pump(.04)
        assert remote_goals, 'RViz goal not forwarded'
        goal_id = remote_goals[-1]['goal']; command = [.02,0.,0.]
        pump(.1)
        assert any(m.linear.x == .02 for m in received), 'Fresh acknowledged command not forwarded'
        cancels.publish(Empty()); pump(.04)
        received.clear(); pump(.15)
        assert not any(m.linear.x != 0 for m in received), 'Cancelled goal resumed'
        print('PASS: shadow does not enable; dispatch enables after zero; leased goal/velocity reaches owner; cancel blocks old goal')
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
