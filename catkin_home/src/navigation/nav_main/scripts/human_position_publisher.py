#!/usr/bin/env python3
import mediapipe as mp
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
import math
import sys
from geometry_msgs.msg import Point
import tf2_ros
import numpy as np

FLT_EPSILON = sys.float_info.epsilon


class HumanPositionPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.pose_publisher = rospy.Publisher(
            "/person_pose_odom", PointStamped, queue_size=5
        )
        self.test_pose_publisher = rospy.Publisher(
            "/test_person_pose_odom", PointStamped, queue_size=5
        )
        self.image_subscriber = rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color", Image, self.image_callback
        )
        self.depth_subscriber = rospy.Subscriber(
            "/zed2/zed_node/depth/depth_registered", Image, self.depth_image_callback
        )
        self.info_subscriber = rospy.Subscriber(
            "/zed2/zed_node/depth/camera_info", CameraInfo, self.camera_info_callback
        )

        self.cv_image = None
        self.camera_info = None
        rospy.loginfo("Subscribed to image")
        # Calling the pose solution from MediaPipe
        self.mp_pose = mp.solutions.pose
        # Calling the solution for image drawing from MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.image_pose_pub = rospy.Publisher("/image_pose", Image, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.x, self.y = 0, 0

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def depth_image_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # Function to handle a ROS camera info input.
    def camera_info_callback(self, data):
        self.camera_info = data
        self.info_subscriber.unregister()

    def image_callback(self, data):
        # self.bridge.imgmsg_to_cv2(data,desired_encoding="rgb8")
        self.cv_image = data

    def get_goal_pose(self):
        if self.cv_image is None or self.camera_info is None:
            return

        point3D = Point()
        # Start media pipe algorithm
        with self.mp_pose.Pose(
            min_detection_confidence=0.5, min_tracking_confidence=0.5
        ) as pose:
            # Converting the ROS image to OpenCV
            # image = self.cv_image
            image = self.bridge.imgmsg_to_cv2(self.cv_image, "rgb8")

            # Detecting the pose with the image
            image.flags.writeable = False
            results = pose.process(image)

            # Drawing the pose detection results
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # Running the MediaPipe solution
            self.mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style(),
            )
            # publish image to a topic called image_pose w

            # If it detects skeletons:
            if results.pose_landmarks:
                # Get the coordinates of the middle point between the shoulders
                self.x = (
                    results.pose_landmarks.landmark[12].x
                    + results.pose_landmarks.landmark[11].x
                ) / 2
                self.y = (
                    results.pose_landmarks.landmark[12].y
                    + results.pose_landmarks.landmark[11].y
                ) / 2
                self.x = int(self.x * image.shape[1])
                self.y = int(self.y * image.shape[0])
                point2D = [self.x, self.y]
                if len(self.depth_image) != 0:
                    depth = self.get_depth(self.depth_image, point2D)
                    point3D_ = self.deproject_pixel_to_point(
                        self.camera_info, point2D, depth
                    )
                    point3D.x = point3D_[2]
                    point3D.y = point3D_[0]
                    point3D.z = point3D_[1]
                    point_x = PointStamped()
                    point_x.header.frame_id = "base_footprint"
                    point_x.point.x = point3D.x
                    point_x.point.y = -point3D.y
                    point_x.point.z = 0

                    print(point_x)
                    # try:
                    #     self.tf_buffer.can_transform(
                    #         "odom", "zed2_camera_center", rospy.Time()
                    #     )
                    # except:
                    #     rospy.logerr(
                    #         "Transformation from camera_frame to odom_frame is not available"
                    #     )
                    # rospy.signal_shutdown("Transformation not available")

                    # Convert the object pose from the body frame to the map frame
                    try:
                        # if self.tf_buffer.can_transform(
                        #     "odom", "zed2_camera_center", rospy.Time()
                        # ):
                        #     object_pose_map = self.tf_buffer.transform(
                        #         point_x, "odom", rospy.Duration(1)
                        #     )
                        test_point_x = point_x
                        test_point_x.point.x -= 0.2
                        if test_point_x.point.x < 0.3:
                            return
                        self.pose_publisher.publish(test_point_x)
                        self.test_pose_publisher.publish(point_x)
                            # print("success")
                        # else:
                        #     print("can't transform")
                    except:
                        print("Failed to transform object pose from camera_frame to odom_frame")
                        rospy.logerr(
                            "Failed to transform object pose from camera_frame to odom_frame"
                        )
                        # rospy.signal_shutdown("Transformation failed")

            image = cv2.circle(image, (self.x, self.y), 20, (255, 0, 0), 2)
            image_pose = Image()
            image_pose.header.stamp = rospy.Time.now()
            image_pose.header.frame_id = "zed2_camera_center"
            image_pose.height = image.shape[0]
            image_pose.width = image.shape[1]
            image_pose.encoding = "rgb8"
            image_pose.is_bigendian = 0
            image_pose.step = image.shape[1] * 3
            image_pose.data = image.tobytes()
            # convert image to ros image
            self.image_pose_pub.publish(image_pose)

    def get_depth(self, depth_image, pixel):
        """
        Given pixel coordinates in an image, the actual image and its depth frame, compute the corresponding depth.
        """
        depth_array = np.array(depth_image, dtype=np.float32)
        heightDEPTH, widthDEPTH = (depth_array.shape[0], depth_array.shape[1])

        x = int(pixel[0])
        y = int(pixel[1])

        def medianCalculation(x, y, width, height, depth_array):
            medianArray = []
            requiredValidValues = 20

            def spiral(
                medianArray,
                depth_array,
                requiredValidValues,
                startX,
                startY,
                endX,
                endY,
                width,
                height,
            ):
                if startX < 0 and startY < 0 and endX > width and endY > height:
                    return
                # Check first and last row of the square spiral.
                for i in range(startX, endX + 1):
                    if i >= width:
                        break
                    if startY >= 0 and math.isfinite(depth_array[startY][i]):
                        medianArray.append(depth_array[startY][i])
                    if (
                        startY != endY
                        and endY < height
                        and math.isfinite(depth_array[endY][i])
                    ):
                        medianArray.append(depth_array[endY][i])
                    if len(medianArray) > requiredValidValues:
                        return
                # Check first and last column of the square spiral.
                for i in range(startY + 1, endY):
                    if i >= height:
                        break
                    if startX >= 0 and math.isfinite(depth_array[i][startX]):
                        medianArray.append(depth_array[i][startX])
                    if (
                        startX != endX
                        and endX < width
                        and math.isfinite(depth_array[i][endX])
                    ):
                        medianArray.append(depth_array[i][endX])
                    if len(medianArray) > requiredValidValues:
                        return
                # Go to the next outer square spiral of the depth pixel.
                spiral(
                    medianArray,
                    depth_array,
                    requiredValidValues,
                    startX - 1,
                    startY - 1,
                    endX + 1,
                    endY + 1,
                    width,
                    height,
                )

            # Check square spirals around the depth pixel till requiredValidValues found.
            spiral(
                medianArray, depth_array, requiredValidValues, x, y, x, y, width, height
            )
            if len(medianArray) == 0:
                return float("NaN")

            # Calculate Median
            medianArray.sort()
            return medianArray[len(medianArray) // 2]

        # Get the median of the values around the depth pixel to avoid incorrect readings.
        return medianCalculation(x, y, widthDEPTH, heightDEPTH, depth_array)

    def deproject_pixel_to_point(self, cv_image_rgb_info, pixel, depth):
        """
        Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients,
        compute the corresponding point in 3D space relative to the same camera
        Reference: https://github.com/IntelRealSense/librealsense/blob/e9f05c55f88f6876633bd59fd1cb3848da64b699/src/rs.cpp#L3505
        """

        def CameraInfoToIntrinsics(cameraInfo):
            intrinsics = {}
            intrinsics["width"] = cameraInfo.width
            intrinsics["height"] = cameraInfo.height
            intrinsics["ppx"] = cameraInfo.K[2]
            intrinsics["ppy"] = cameraInfo.K[5]
            intrinsics["fx"] = cameraInfo.K[0]
            intrinsics["fy"] = cameraInfo.K[4]
            if cameraInfo.distortion_model == "plumb_bob":
                intrinsics["model"] = "RS2_DISTORTION_BROWN_CONRADY"
            elif cameraInfo.distortion_model == "equidistant":
                intrinsics["model"] = "RS2_DISTORTION_KANNALA_BRANDT4"
            intrinsics["coeffs"] = [i for i in cameraInfo.D]
            return intrinsics

        # Parse ROS CameraInfo msg to intrinsics dictionary.
        intrinsics = CameraInfoToIntrinsics(cv_image_rgb_info)

        # Cannot deproject from a forward-distorted image
        if intrinsics["model"] == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY":
            return

        x = (pixel[0] - intrinsics["ppx"]) / intrinsics["fx"]
        y = (pixel[1] - intrinsics["ppy"]) / intrinsics["fy"]

        xo = x
        yo = y

        if intrinsics["model"] == "RS2_DISTORTION_INVERSE_BROWN_CONRADY":
            # need to loop until convergence
            # 10 iterations determined empirically
            for i in range(10):
                r2 = float(x * x + y * y)
                icdist = float(1) / float(
                    1
                    + (
                        (intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2
                        + intrinsics["coeffs"][0]
                    )
                    * r2
                )
                xq = float(x / icdist)
                yq = float(y / icdist)
                delta_x = float(
                    2 * intrinsics["coeffs"][2] * xq * yq
                    + intrinsics["coeffs"][3] * (r2 + 2 * xq * xq)
                )
                delta_y = float(
                    2 * intrinsics["coeffs"][3] * xq * yq
                    + intrinsics["coeffs"][2] * (r2 + 2 * yq * yq)
                )
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist

        if intrinsics["model"] == "RS2_DISTORTION_BROWN_CONRADY":
            # need to loop until convergence
            # 10 iterations determined empirically
            for i in range(10):
                r2 = float(x * x + y * y)
                icdist = float(1) / float(
                    1
                    + (
                        (intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2
                        + intrinsics["coeffs"][0]
                    )
                    * r2
                )
                delta_x = float(
                    2 * intrinsics["coeffs"][2] * x * y
                    + intrinsics["coeffs"][3] * (r2 + 2 * x * x)
                )
                delta_y = float(
                    2 * intrinsics["coeffs"][3] * x * y
                    + intrinsics["coeffs"][2] * (r2 + 2 * y * y)
                )
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist

        if intrinsics["model"] == "RS2_DISTORTION_KANNALA_BRANDT4":
            rd = float(math.sqrt(x * x + y * y))
            if rd < FLT_EPSILON:
                rd = FLT_EPSILON

            theta = float(rd)
            theta2 = float(rd * rd)
            for i in range(4):
                f = float(
                    theta
                    * (
                        1
                        + theta2
                        * (
                            intrinsics["coeffs"][0]
                            + theta2
                            * (
                                intrinsics["coeffs"][1]
                                + theta2
                                * (
                                    intrinsics["coeffs"][2]
                                    + theta2 * intrinsics["coeffs"][3]
                                )
                            )
                        )
                    )
                    - rd
                )
                if abs(f) < FLT_EPSILON:
                    break
                df = float(
                    1
                    + theta2
                    * (
                        3 * intrinsics["coeffs"][0]
                        + theta2
                        * (
                            5 * intrinsics["coeffs"][1]
                            + theta2
                            * (
                                7 * intrinsics["coeffs"][2]
                                + 9 * theta2 * intrinsics["coeffs"][3]
                            )
                        )
                    )
                )
                theta -= f / df
                theta2 = theta * theta
            r = float(math.tan(theta))
            x *= r / rd
            y *= r / rd

        if intrinsics["model"] == "RS2_DISTORTION_FTHETA":
            rd = float(math.sqrt(x * x + y * y))
            if rd < FLT_EPSILON:
                rd = FLT_EPSILON
            r = (float)(
                math.tan(intrinsics["coeffs"][0] * rd)
                / math.atan(2 * math.tan(intrinsics["coeffs"][0] / float(2.0)))
            )
            x *= r / rd
            y *= r / rd

        return (depth * x, depth * y, depth)


if __name__ == "__main__":
    rospy.logwarn("Starting depth")
    rospy.init_node("detector_humano", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    human_position_pub = HumanPositionPublisher()
    try:
        while not rospy.is_shutdown():
            human_position_pub.get_goal_pose()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("Keyboard interrupt detected, stopping listener")
