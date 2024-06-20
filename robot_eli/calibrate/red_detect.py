import cv2
import numpy as np
import pyrealsense2 as rs
from elite_robot import Robot
import keyboard
# 初始化RealSense相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
tcp_host_ip = "192.168.199.100"  # IP and port to robot arm as TCP client (UR5)
tcp_port = 8055
is_use_robotiq85 = False
tool_xyz = np.asarray([0.5, 0.1, 0.2])

tool_orientation = np.asarray([-180*(np.pi/180), 0, 0])
# ---------------------------------------------

# Move robot to home pose
robot = Robot(tcp_host_ip, tcp_port)
# robot.move_j([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, np.pi])
grasp_home = np.append(
    tool_xyz, tool_orientation
)  # [-0.1, 0.25, 0.3, -np.pi, 0, 0]  # you can change me
pipeline.start(config)
robot.move_j_p(grasp_home)
robot.open_gripper(speed=100, force=30)
# Slow down robot
robot.joint_acc = 0.5
robot.joint_vel = 0.5

try:
    while True:
        # 获取相机帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # 将帧转换为OpenCV格式
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 在HSV颜色空间中查找红色物体
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 找到红色物体的轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 找到红色物体的轮廓并计算质心坐标
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                # 计算轮廓的质心坐标
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # 在图像上标记质心点
                    cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)

                    # # 输出质心坐标
                    # print("Centroid Coordinates (Pixel):", cx, cy)

                    # 对齐相机的深度和彩色图像
                    align = rs.align(rs.stream.color)
                    aligned_frames = align.process(frames)
                    depth_frame_aligned = aligned_frames.get_depth_frame()
                    # 获取相机内参
                    depth_intrinsics = rs.video_stream_profile(
                        depth_frame_aligned.profile).get_intrinsics()

                    # 将质心像素坐标转换为相机内参下的相机坐标
                    depth = depth_frame_aligned.get_distance(cx, cy)
                    if depth != 0:  # 如果深度不为零，则有效
                        camera_coordinate = rs.rs2_deproject_pixel_to_point(
                            depth_intrinsics, [cx, cy], depth)
                        # print("Object Coordinates (Camera):", camera_coordinate)
                        camera_point = np.array([cx, cy, depth])
                        # print("camera_point (Camera):", camera_point)
                        # 将相机坐标系下的点表示为齐次坐标形式
                        homogeneous_point = np.append(camera_point, 1)
                        # 手眼标定的变换矩阵
                        T_handeye = np.array([
                            [-0.05352997328436295, -0.9985611699462507, -
                                0.0031830544686656074, 0.18336193295548398],
                            [-0.9977912071841648, 0.0536137198311533, -
                                0.03922086067176618, 1.0230365161208188],
                            [0.03933508390918808, 0.0010765321368736377, -
                                0.9992254961981383, 0.16560628636516092],
                            [0.0, 0.0, 0.0, 1.0]
                        ])
                        # 使用变换矩阵进行坐标转换
                        arm_point = np.dot(T_handeye, homogeneous_point)
                        # 将结果转换回非齐次坐标形式
                        arm_point = arm_point[:3] / arm_point[3]
                        target_position11 = np.asarray(
                            [
                                arm_point[0]/1000,
                                arm_point[1]/1000,
                                arm_point[2]/1000,
                            ]
                        )

        if keyboard.is_pressed('space'):

            print("空格键被按下")
            # 提取机械臂坐标系下的点
            x_m, y_m, z_m = target_position11
            print("ARM:", x_m, y_m, z_m)
            target_pos = np.append(
                target_position11, tool_orientation)
            print(target_pos)
            # 运动到待抓取位置
            robot.move_j_p(target_pos)
            # 回到预抓取位置
            robot.move_j_p(grasp_home)
        # 显示结果
        cv2.imshow('Red Object Detection', color_image)
        if cv2.waitKey(1) == ord("c"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
