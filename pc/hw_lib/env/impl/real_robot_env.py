import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu
from std_srvs.srv import Trigger
from hw_lib.env.env import RobotEnv, ImuData, ServoPositionData, CameraData, Observation
import json


class RealRobotEnv(RobotEnv):
    def __init__(self):
        try:
            rospy.init_node('real_robot_env', anonymous=True)
        except rospy.exceptions.ROSException:
            pass # ignore 'rospy.init_node() has already been called' error

        # ---- Wait for action_server ----
        rospy.loginfo("Waiting for action_server to be available...")
        try:
            rospy.wait_for_service('/robot/get_servo_positions', timeout=5.0)
            rospy.loginfo("action_server is running. RealRobotEnv initialized successfully.")
        except rospy.ROSException:
            error_msg = "Could not connect to action_server. Please ensure the node is running."
            rospy.logerr(error_msg)
            raise RuntimeError(error_msg)

        # ---- Service Proxies for Servo Data ----
        self._get_servo_positions_srv = rospy.ServiceProxy('/robot/get_servo_positions', Trigger)

        # ---- Subscribers for Sensor Data ----
        self._bridge = CvBridge()
        self._latest_imu_data = None
        self._latest_image_data = None

        # NOTE: These topic names are common defaults and may need to be adjusted
        self._imu_sub = rospy.Subscriber('/imu/data', Imu, self._imu_callback)
        self._camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self._camera_callback)

        rospy.loginfo("RealRobotEnv fully initialized.")

    def _imu_callback(self, msg):
        self._latest_imu_data = msg

    def _camera_callback(self, msg):
        try:
            self._latest_image_data = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def _get_servo_data(self) -> ServoPositionData:
        """Calls services to get servo positions and voltages."""
        try:
            positions_resp = self._get_servo_positions_srv()
            positions = json.loads(positions_resp.message) if positions_resp.success else {}
            return ServoPositionData(data=positions)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return ServoPositionData(data={})

    def _get_imu_data(self) -> ImuData:
        """Returns the latest IMU data."""
        if self._latest_imu_data:
            imu_dict = {
                'orientation_x': self._latest_imu_data.orientation.x,
                'orientation_y': self._latest_imu_data.orientation.y,
                'orientation_z': self._latest_imu_data.orientation.z,
                'orientation_w': self._latest_imu_data.orientation.w,
                'angular_velocity_x': self._latest_imu_data.angular_velocity.x,
                'angular_velocity_y': self._latest_imu_data.angular_velocity.y,
                'angular_velocity_z': self._latest_imu_data.angular_velocity.z,
                'linear_acceleration_x': self._latest_imu_data.linear_acceleration.x,
                'linear_acceleration_y': self._latest_imu_data.linear_acceleration.y,
                'linear_acceleration_z': self._latest_imu_data.linear_acceleration.z,
            }
            return ImuData(data=imu_dict)
        return ImuData(data={})

    def _get_camera_data(self) -> CameraData:
        """Returns the latest camera image as a numpy array."""
        if self._latest_image_data is not None:
            return CameraData(data=self._latest_image_data)
        # Return an empty image if no data is available
        return CameraData(data=np.zeros((480, 640, 3), dtype=np.uint8))

    def get_observation(self) -> Observation:
        return {
            "imu": self._get_imu_data(),
            "servo": self._get_servo_data(),
            "camera": self._get_camera_data(),
        }

    def step(self, action):
        pass
