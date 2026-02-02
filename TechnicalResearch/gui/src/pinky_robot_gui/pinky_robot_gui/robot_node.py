import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus


class SimpleRobotNode(Node):
    """간소화된 ROS2 로봇 노드"""
    
    def __init__(self):
        super().__init__('robot_gui_controller')
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # 현재 위치
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 위치 콜백
        self.position_callback = None
        
        self.get_logger().info('🤖 Robot Controller 초기화 완료')
    
    def set_position_callback(self, callback):
        """위치 업데이트 콜백 설정"""
        self.position_callback = callback
    
    def odom_callback(self, msg):
        """Odometry 콜백"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Quaternion → Yaw
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # GUI 콜백 호출
        if self.position_callback:
            self.position_callback(self.current_x, self.current_y, self.current_yaw)
    
    def emergency_stop(self):
        """긴급 정지"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn('🚨 긴급 정지!')
    
    def navigate_to_pose(self, x, y, yaw):
        """Nav2를 이용한 목표 지점 이동"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('❌ Nav2 서버 응답 없음!')
            return False
        
        from geometry_msgs.msg import PoseStamped
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Yaw → Quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        self.get_logger().info(f'📍 목표 지점: ({x:.2f}, {y:.2f}), {math.degrees(yaw):.1f}°')
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        
        return True
    
    def nav_goal_response_callback(self, future):
        """목표 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ 목표 거부됨')
            return
        
        self.get_logger().info('✅ 목표 수락됨')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """내비게이션 피드백"""
        pass
    
    def nav_result_callback(self, future):
        """내비게이션 결과"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎯 목표 도착!')
        else:
            self.get_logger().warn(f'⚠️ 내비게이션 실패 (상태: {status})')
