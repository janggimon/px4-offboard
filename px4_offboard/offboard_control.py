import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')

        self.bridge = CvBridge()

        # 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/downward_camera/image_raw',
            self.image_callback,
            10)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()

        # ArUco 마커 포즈 퍼블리시
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/drone/aruco_pose', 10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i in range(len(ids)):
                corner = corners[i][0]
                center_x = (corner[0][0] + corner[2][0]) / 2.0
                center_y = (corner[0][1] + corner[2][1]) / 2.0

                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'aruco_marker'
                pose_msg.pose.position.x = float(center_x)
                pose_msg.pose.position.y = float(center_y)
                pose_msg.pose.position.z = 0.0

                # 퍼블리시
                self.pose_publisher.publish(pose_msg)

        cv2.imshow('ArUco Marker Detection', cv_image)
        cv2.waitKey(1)


class DroneControllerNode(Node):
    def __init__(self):
        super().__init__('drone_controller_node')

        # ArUco 마커 포즈 구독
        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/drone/aruco_pose', self.aruco_pose_callback, 10)

        # PX4 오프보드 제어 퍼블리셔
        self.offboard_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # 주기적 명령 전송을 위한 타이머
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        # Threshold 설정 (중앙에서 허용되는 오차 범위)
        self.x_threshold = 20.0
        self.y_threshold = 20.0

        # 목표 위치 (카메라 화면의 중앙 좌표)
        self.target_x = 320.0
        self.target_y = 240.0

        # 착륙 상태 플래그
        self.landing = False

    def aruco_pose_callback(self, pose_msg):
        if self.landing:
            return  # 이미 착륙 중이라면 추가 조작을 하지 않음

        marker_x = pose_msg.pose.position.x
        marker_y = pose_msg.pose.position.y

        error_x = marker_x - self.target_x
        error_y = marker_y - self.target_y

        if abs(error_x) > self.x_threshold or abs(error_y) > self.y_threshold:
            # 중앙에 도달하지 않으면 위치 보정
            self.publish_offboard_control(error_x, error_y)
        else:
            # 중앙에 도달하면 착륙 명령 전송
            self.publish_descent_control()

    def publish_offboard_control(self, error_x, error_y):
        # 중앙에 맞추기 위한 위치 보정 명령
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [float(-0.01 * error_x), float(-0.01 * error_y), 0.0]  # 수평 위치 보정
        trajectory_msg.velocity = [0.0, 0.0, 0.0]
        self.trajectory_publisher.publish(trajectory_msg)

    def publish_descent_control(self):
        # 착륙 명령 전송
        self.get_logger().info('Landing initiated...')
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0, 0.0, -0.2]  # 수직 착륙
        trajectory_msg.velocity = [0.0, 0.0, -0.2]
        self.get_logger().info(f"Publishing TrajectorySetpoint: {trajectory_msg.position}")
        self.trajectory_publisher.publish(trajectory_msg)

        # 착륙 중 상태 플래그 설정
        self.landing = True

    def cmdloop_callback(self):
        self.get_logger().info('cmdloop_callback called')
    
        # Offboard Control Mode 설정
        offboard_mode_msg = OffboardControlMode()
        offboard_mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_mode_msg.position = True
        offboard_mode_msg.velocity = False
        offboard_mode_msg.acceleration = False
        offboard_mode_msg.attitude = False
        offboard_mode_msg.body_rate = False
        self.offboard_mode_publisher.publish(offboard_mode_msg)

        # Trajectory Setpoint 송신
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        trajectory_msg.position = [0.0, 0.0, -1.0]  # 예시로 수직 방향으로 내려가도록 설정
        trajectory_msg.velocity = [0.0, 0.0, 0.0]
        trajectory_msg.acceleration = [0.0, 0.0, 0.0]
        self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    detector = ArucoMarkerDetector()
    controller = DroneControllerNode()

    # 멀티스레드 실행
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    detector.destroy_node()
    controller.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
