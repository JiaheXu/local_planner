"""Run the existing XPolicy leased relay and checked auto-enable on the PC."""
import json
import os
from pathlib import Path
import signal
import sys
import time

robot = Path(os.environ.get('XPOLICY_ROOT', '/home/ubuntu/xpolicylabdagger')) / 'real_world/robots/astribot_s1'
sys.path.insert(0, str(robot))
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool
from planner_orin_bridge import PlannerBridge
from planner_transport import REMOTE, CommandLease
from auto_base_enable import AutoBaseEnable


class DiagnosedLease(CommandLease):
    """Record the existing validator's outcome without changing acceptance."""
    def __init__(self):
        super().__init__()
        self.last_forwarded = -float('inf')
        self.last_rejection = None

    def accept(self, envelope, now):
        try:
            command = super().accept(envelope, now)
        except (ValueError, TypeError) as exc:
            self.last_rejection = str(exc)
            raise
        self.last_forwarded = now
        self.last_rejection = None
        return command


class DispatchBridge:
    def __init__(self, source_domain=25, output_domain=0):
        self.contexts, self.nodes, self.executors = [], [], []
        for domain, suffix in ((source_domain, 'source'), (output_domain, 'output')):
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
            node = Node('astribot_orin_dispatch_' + suffix, context=context, use_global_arguments=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.contexts.append(context); self.nodes.append(node); self.executors.append(executor)
        self.source, self.output = self.nodes
        self.bridge = PlannerBridge(self.source, self.output, control=True)
        self.bridge.lease = DiagnosedLease()
        self.next_diagnostic = 0.
        self.owner, self.planner = {}, {}
        self.owner_received = self.planner_received = -float('inf')
        self.output.create_subscription(String, '/astribot/base/status', self.owner_status, 1)
        self.source.create_subscription(String, REMOTE + '/status', self.planner_status, 1)
        self.enable = self.output.create_client(SetBool, '/astribot/base/enable')
        self.auto_enable = AutoBaseEnable(lambda: self.enable.call_async(SetBool.Request(data=True)),
            self.output.get_logger().info, started=time.monotonic())

    def owner_status(self, message):
        try:
            status = json.loads(message.data)
            self.owner = status if isinstance(status, dict) else {}
        except ValueError:
            self.owner = {}
        self.owner_received = time.monotonic()

    def planner_status(self, message):
        try:
            status = json.loads(message.data)
            self.planner = status if isinstance(status, dict) else {}
        except ValueError:
            self.planner = {}
        self.planner_received = time.monotonic()

    def spin_once(self):
        for executor in self.executors:
            executor.spin_once(timeout_sec=.002)
        now = time.monotonic()
        # Only an idle, fresh dispatch coordinator can trigger re-enable. The
        # existing owner independently requires fresh zero commands and feedback.
        idle = (0 <= now-self.planner_received <= .5 and self.planner.get('dispatch') is True
                and self.planner.get('active') is False and not self.bridge.conflict)
        self.auto_enable.tick(now, status=self.owner, age=now-self.owner_received,
            publishers=self.output.count_publishers('/cmd_vel'), idle=idle,
            service_ready=self.enable.service_is_ready())
        if now >= self.next_diagnostic:
            def age(received):
                return None if received == -float('inf') else round(now-received, 3)
            report = dict(
                cmd_vel_publishers=self.output.count_publishers('/cmd_vel'),
                orin_command_publishers=self.source.count_publishers(REMOTE+'/command'),
                owner_publishers=self.output.count_publishers('/astribot/base/status'),
                owner_age_s=age(self.owner_received), planner_age_s=age(self.planner_received),
                forwarded_age_s=age(self.bridge.lease.last_forwarded),
                rejection=self.bridge.lease.last_rejection, transport_conflict=self.bridge.conflict,
                enable_service_ready=self.enable.service_is_ready(),
                enable_status=self.auto_enable.last_message,
                owner_reason=self.owner.get('reason'), planner_dispatch=self.planner.get('dispatch'))
            self.output.get_logger().info('Dispatch relay: ' + json.dumps(report))
            self.next_diagnostic = now + 5.

    def close(self):
        for executor in self.executors: executor.shutdown()
        for node in self.nodes: node.destroy_node()
        for context in self.contexts: context.try_shutdown()


def main():
    bridge = DispatchBridge()
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, stop)
    try:
        while not stopped: bridge.spin_once()
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
