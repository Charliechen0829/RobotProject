from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from colorama import init, Fore, Style
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection2DArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

# 初始化 colorama 
init(autoreset=True)

class ResultSubscriber(Node):
    def __init__(self):
        super().__init__('result_subscriber')
        self.group=ReentrantCallbackGroup()
        self.subscription = self.create_subscription(Detection2DArray, '/yolo_result', self.listener_callback, 10,callback_group=self.group)
        self.find_person = False

        self.navigator = BasicNavigator()
        # 等待导航启动完成
        self.navigator.waitUntilNav2Active()

        self.waypoint = [[4.42,6.0,1.0],[7.56,0.16,1.0],[7.94,6.95,-1.0],[-0.41,8.62,1.0],[0.0,0.0,1.0]]
        self.point_index = 0
        self.person_point, self.return_point = [6.1, 7.5], [0.0, 0.0]

        # 使用线程启动巡航
        self.patrolling=self.create_timer(2,self.start_patrolling,callback_group=self.group)
        self.executor=MultiThreadedExecutor(2)

    def start_patrolling(self):
        if not self.find_person and self.navigator.isTaskComplete():
            point = self.waypoint[self.point_index]
            self.nav2pose(point[0], point[1], point[2])
            self.point_index = (self.point_index + 1) % len(self.waypoint)

    def nav2pose(self, x, y, w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w

        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.navigator.get_logger().info(
                f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s 后到达')
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelTask()
                break
            time.sleep(1.5)  # 避免过于频繁地循环
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigator.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.navigator.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.navigator.get_logger().error('导航结果：失败')
        else:
            self.navigator.get_logger().error('导航结果：返回状态无效')
        return result

    def listener_callback(self, msg: Detection2DArray):
        if self.find_person:
            return

        for detection in msg.detections:
            object_name = detection.results[0].hypothesis.class_id
            if object_name == 'person':
                self.find_person = True
                self.navigator.cancelTask()
                # discover
                self.printContext('Robot discovers person! Robot is on its way!')
                self.nav2pose(self.person_point[0], self.person_point[1], 1.0)
                # returning
                self.printContext('           The robot is returning!           ')
                result=self.nav2pose(self.return_point[0], self.return_point[1], 1.0)
                if result==TaskResult.SUCCEEDED:
                    self.printContext('Successfully returned')
                rclpy.shutdown()
                return

    def printContext(self, context):
        space_len = 10
        str = '#' + ' ' * space_len + context + ' ' * space_len + '#'
        print(Fore.GREEN + '#' * len(str))
        print(Fore.GREEN + str)
        print(Fore.GREEN + '#' * len(str))
        print(Style.RESET_ALL)

def main():
    rclpy.init()
    result_subscriber = ResultSubscriber()
    rclpy.spin(result_subscriber)
    result_subscriber.destroy_node()
    rclpy.shutdown()
        

if __name__ == "__main__":
    main()
