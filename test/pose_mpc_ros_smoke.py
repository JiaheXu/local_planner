"""Opt-in real C++/ROS pose MPC protocol test; isolated localhost domain 223 only."""
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point
from local_planner.msg import MpcRequest, MpcResult
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from pose_mpc import validate_pose_prediction


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='223' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Use isolated localhost ROS domain 223')
    binary=Path(get_package_prefix('local_planner'))/'lib/local_planner/localPlanner'
    server=subprocess.Popen([str(binary),'--ros-args','-p','planner_backend:=mpc',
                             '-p','pathFolder:=/tmp/no-candidate-assets-needed'],
                            stdout=subprocess.DEVNULL)
    node=None
    try:
        rclpy.init(args=[]);node=Node('pose_mpc_protocol_test');results=[]
        pub=node.create_publisher(MpcRequest,'/mpc_request',1)
        node.create_subscription(MpcResult,'/mpc_result',results.append,10)
        def wait(predicate,seconds=5.):
            end=time.monotonic()+seconds
            while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            assert predicate(),'MPC protocol timeout'
        wait(lambda:pub.get_subscription_count()==1 and node.count_publishers('/mpc_result')==1)
        for index,(xy,yaw,mode,obstacle) in enumerate([
                ((.5,.2),1.,True,False),((.05,0.),.08,True,False),
                ((0.,0.),-.2,True,False),((.2,0.),0.,False,False),
                ((.2,0.),1.,True,True),((2.,0.),.8,True,False),
                ((2.,0.),.8,True,'distant_obstacle')]):
            req=MpcRequest();req.header.frame_id='astribot_viz_chassis'
            req.header.stamp=node.get_clock().now().to_msg();req.sent_at=req.header.stamp
            req.epoch=index+1;req.goal.x,req.goal.y=xy;req.plan_yaw=mode;req.goal_yaw=yaw
            if obstacle == 'distant_obstacle':
                req.obstacles=[Point(x=1.+.1*math.cos(a),y=.1*math.sin(a))
                               for a in [i*math.pi/30 for i in range(60)]]
            elif obstacle:req.obstacles=[Point(x=.5)]
            pub.publish(req);wait(lambda:len(results)>index)
            result=results[index]
            assert result.header==req.header and result.epoch==req.epoch
            if obstacle is True:
                assert result.success and result.blocked and len(result.path.poses)==1
                assert (result.command.linear.x,result.command.linear.y,result.command.angular.z)==(0.,0.,0.)
                continue
            assert result.success,result.error
            assert not result.blocked
            if obstacle == 'distant_obstacle':
                # Compare with the same direct-goal solve, including simultaneous yaw.
                assert result.command==results[index-1].command
                assert [p.pose for p in result.path.poses]==[p.pose for p in results[index-1].path.poses]
            if mode:
                headings=validate_pose_prediction(result.path,result.command,(0.,0.,0.))
                assert headings[-1]*yaw>0. and result.command.angular.z*yaw>0.
            else:assert result.command.angular.z==0. and all(p.pose.orientation.w==1. for p in result.path.poses)
        print('PASS: real ROS pose request/result, trajectory yaw, slowdown, pure turn, legacy XY and obstacle blocked stop; no hardware')
    finally:
        if node is not None:node.destroy_node();rclpy.try_shutdown()
        server.terminate();server.wait(timeout=5)


if __name__=='__main__':main()
