import cv2
import numpy as np
import pyrealsense2 as rs
from elite_robot import Robot
import math
import time
import torch
from pathlib import Path
import random
from ultralytics import YOLO
import keyboard
import threading

tcp_host_ip = "192.168.199.100"  # IP and port to robot arm as TCP client (UR5)
tcp_port = 8055
# is_use_robotiq85 = False


# 标定参数
# R_tc = ([[-0.9998, -0.0177, 0.0083],
#          [0.0177, -0.9998, -0.0087],
#          [0.0085, -0.0086, 0.9999]])
R_tc = ([[0.9981, 0.0497, -0.0910],
         [-0.0536, 0.9965, -0.063],
         [0.0875, 0.0677, 0.9939]])
# ToolinCamPose平移矩阵
# T_tc = [33.1085, 70.5922, 35.3309]  # 毫米
T_tc = [12.2079, 62.1097, 17.8317]  # 毫米

tool_orientation = np.asarray([-np.pi, 0, 0])
tool_xyz = np.asarray([0.5, 0, 0.55])
photo_ready_waypoint = (0.5, 0, 0.55, -np.pi, 0, 0)  # 初始位姿 米 弧度

tool_offset = np.asarray([0.00, 0.00, 0.2])  # 米
# ---------------------------------------------


def get_photo_pose():
    robot = Robot(tcp_host_ip, tcp_port)
    current_pos = robot.get_current_pose()
    # print("Current Robot Pose:", current_pos)

    roll_rad = current_pos[3]
    pitch_rad = current_pos[4]
    yaw_rad = current_pos[5]
    print(roll_rad, pitch_rad, yaw_rad)
    # Compute rotation matrix
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll_rad), -math.sin(roll_rad)],
                    [0, math.sin(roll_rad), math.cos(roll_rad)]])

    R_y = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)],
                    [0, 1, 0],
                    [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])

    R_z = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0],
                    [math.sin(yaw_rad), math.cos(yaw_rad), 0],
                    [0, 0, 1]])

    # print(R_x, R_y, R_z)
    # Combine rotation matrices
    r_end_to_base = np.dot(R_z, np.dot(R_y, R_x))
    t_end_to_base = np.array(
        [current_pos[0], current_pos[1], current_pos[2]]).T
    R_eb = r_end_to_base
    T_eb = t_end_to_base
    return R_eb, T_eb


def camera_to_base(x_c, y_c, z_c, R_eb, T_eb):

    P_in_cam = np.array([x_c, y_c, z_c]).T
    # 坐标转换
    P_in_end = np.matmul(R_tc, P_in_cam)
    P_in_end = P_in_end + np.array(T_tc).T
    # P_in_base = np.dot(self.R_eb, P_in_end)
    P_in_base = np.matmul(R_eb, P_in_end)
    P_in_base = P_in_base + np.array(T_eb).T
    HandP_in_base = P_in_base/1000
    return HandP_in_base


def pick_action(tmp_x, tmp_y, tmp_z):
    robot = Robot(tcp_host_ip, tcp_port)
    robot.move_j_p(photo_ready_waypoint)
    robot.open_gripper(speed=100, force=30)
    time.sleep(1)
    R_eb, T_eb = get_photo_pose()

    target_position = camera_to_base(tmp_x, tmp_y, tmp_z, R_eb, T_eb)
    # print("target_position", target_position)
    xyz = target_position
    target_rxyz = [-180, 0, 0]
    click_point = np.array(
        [tmp_x/1000, tmp_y/1000, tmp_z/1000])
    print("click_point", click_point)
    ###################### 机械臂前进####################
    if xyz is not None:
        pos1 = (xyz[0], xyz[1], xyz[2])

        # print("pos", type(pos1))
        rpy_xyz_rad = [i / 180.0 * math.pi for i in target_rxyz]
        # print("rpy_xyz_rad", type(rpy_xyz_rad))
        current_pos = robot.get_joint_pose()
        # print("Joint Pose", type(current_pos))
        # 设置工具的偏移量，并计算机械臂的逆解
        # offset = (tool_offset[0], tool_offset[1], tool_offset[2])
        # pre_user_tool = dict(pos=offset, rad=(0, 0, 0))
        # pre_pos_rad = dict(
        #     pos=(pre_user_tool["pos"][0] + pos1[0], pre_user_tool["pos"]
        #          [1] + pos1[1], pre_user_tool["pos"][2] + pos1[2]),
        #     rad=(pre_user_tool["rad"][0] + rpy_xyz_rad[0], pre_user_tool["rad"]
        #          [1] + rpy_xyz_rad[1], pre_user_tool["rad"][2] + rpy_xyz_rad[2])
        # )
        cam_pose = np.loadtxt(
            "/home/nx/桌面/test/robot_e/camera_pose_tool.txt",
            delimiter=" ",
        )
        click_point.shape = (3, 1)
        camera2tool = cam_pose
        tool2base = robot.rpy2R(tool_orientation)
        tool2base = np.asarray(
            [
                [tool2base[0, 0], tool2base[0, 1], tool2base[0, 2], tool_xyz[0]],
                [tool2base[1, 0], tool2base[1, 1], tool2base[1, 2], tool_xyz[1]],
                [tool2base[2, 0], tool2base[2, 1], tool2base[2, 2], tool_xyz[2]],
                [0, 0, 0, 1],
            ]
        )

        camera_target_position = np.asarray(
            [[click_point[0]], [click_point[1]], [click_point[2]], [1.0]], dtype=object
        )
        target_position1 = np.dot(
            np.dot(tool2base[0:4, 0:4], camera2tool[0:4, 0:4]
                   ), camera_target_position
        )
        target_position1 = np.concatenate(target_position1[0:3, 0])
        # 人为补偿
        if -target_position1[1] > 0:
            target_position11 = np.asarray(
                [
                    target_position1[0]+0.035,
                    -target_position1[1]-0.03,
                    target_position1[2] - 0.70,
                ]
            )
            print("y补偿")
        else:
            target_position11 = np.asarray(
                [
                    target_position1[0]+0.04,
                    -target_position1[1]-0.0,
                    target_position1[2] - 0.70,
                ]
            )
        destination = np.append(target_position11, tool_orientation)

        # pre_ik_result = robot.inverse_kin(
        #     current_pos, pre_pos_rad["pos"], pre_pos_rad["rad"])
        pre_ik_result = robot.inverse_kin(
            current_pos, target_position11, tool_orientation)
        print("destination", destination)
        # print("pre_user_tool", pre_user_tool)

        # P1 = [0, -90, 90, -90, 90, 0]
        # pre_ik_result = robot.inverse_kin(P1, [0.5, 0.1, 0.5], [-3.14159, 0, 0]
        #                                   )
        if pre_ik_result is not None:
            # 轴动到目标位置
            pre_ik_result = eval(pre_ik_result)
            # print("pre_ik_result", (pre_ik_result))
            robot.move_j(pre_ik_result)
            robot.close_gripper(speed=150, force=140)
            time.sleep(2)
        else:
            print("逆解失败")


def xywh2xyxy(x):
    y = x.clone()
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # x_min
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # y_min
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # x_max
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # y_max
    return y


def non_max_suppression(prediction, conf_thres=0.5, iou_thres=0.45, classes=None, agnostic=False, max_det=1000):
    output = []
    for det in prediction:
        det_boxes = det.boxes.xywh  # 获取 xywh 格式的边界框数据
        det_boxes_xyxy = xywh2xyxy(det_boxes[:, :4])  # 将 xywh 转换为 xyxy 格式
        scores = det.boxes.conf  # 获取置信度分数
        class_ids = det.boxes.cls  # 获取类别ID

        # 将 xyxy 和置信度分数拼接起来
        det_boxes = torch.cat(
            (det_boxes_xyxy, scores[:, None], class_ids[:, None]), dim=1)

        # print(f"Original det shape: {det_boxes.shape}")  # 打印原始 det 的形状
        # print(f"Original det content: {det_boxes}")  # 打印 det_boxes 的内容

        # 过滤掉低于置信度阈值的检测结果
        mask = scores >= conf_thres
        det_boxes = det_boxes[mask]

        # 置信度分数
        scores = det_boxes[:, 4]
        # 使用 NMS 保留框
        keep = nms(det_boxes[:, :4], scores, iou_thres)

        # 确保不超过 max_det 的数量
        keep = keep[:max_det] if len(keep) > max_det else keep

        # 保存结果
        output.append(det_boxes[keep])
        # print(f"Output shape: {det_boxes[keep].shape}")  # 打印输出的形状
        print(f"Output content: {det_boxes[keep]}")  # 打印输出的内容

    return output


def letterbox(img, new_shape=(640, 640), stride=32, auto=True):
    # Resize and pad image while keeping aspect ratio
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - \
        new_unpad[1]  # width, height padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=(128, 128, 128))  # add border
    return img, ratio, (dw, dh)


def nms(boxes, scores, iou_threshold):
    # 确保 boxes 和 scores 是张量，并转换为 CPU 上的 NumPy 数组
    if isinstance(boxes, torch.Tensor):
        boxes = boxes.cpu().detach().numpy()
    if isinstance(scores, torch.Tensor):
        scores = scores.cpu().detach().numpy()

    # 确保 boxes 和 scores 的长度相同
    assert len(boxes) == len(scores)

    # 如果没有候选框，则返回空列表
    if len(boxes) == 0:
        return []

    # 初始化一个列表来存储保留的框的索引
    keep = []

    # 获取所有框的坐标
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # 计算所有框的面积
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)

    # 对分数进行排序（按降序排列）
    order = scores.argsort()[::-1]

    # 遍历所有框，执行非极大值抑制
    while len(order) > 0:
        # 获取当前分数最高的框的索引
        i = order[0]

        # 将当前框添加到保留列表中
        keep.append(i)

        # 计算当前框与其他所有框的交并比
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)

        intersection = w * h

        iou = intersection / (areas[i] + areas[order[1:]] - intersection)

        # 获取交并比小于阈值的索引
        idx = np.where(iou <= iou_threshold)[0]

        # 更新 order 列表，删除与当前框重叠过多的框
        order = order[idx + 1]

    # 返回保留的框的索引
    return keep


def scale_coords(img_shape, coords, img0_shape):
    # img_shape: 目标图像的形状 (h, w)
    # coords: 框的坐标 (x_min, y_min, x_max, y_max)
    # img0_shape: 原始图像的形状 (h0, w0)
    gain = min(img_shape[0] / img0_shape[0],
               img_shape[1] / img0_shape[1])  # 缩放比例
    pad_x = (img_shape[1] - img0_shape[1] * gain) / 2  # x方向的填充
    pad_y = (img_shape[0] - img0_shape[0] * gain) / 2  # y方向的填充
    coords[:, [0, 2]] -= pad_x  # 调整x坐标
    coords[:, [1, 3]] -= pad_y  # 调整y坐标
    coords[:, :4] /= gain  # 缩放坐标
    return coords


def increment_path(path, exist_ok=False):
    # path: 基本路径
    # exist_ok: 如果为True且路径已经存在，不会引发异常
    path = Path(path)
    while path.exists():
        stem = path.stem  # 基本路径的文件名部分
        suffix = path.suffix  # 基本路径的后缀部分
        stem += '_1'  # 在文件名后添加下划线和数字
        path = path.with_name(stem + suffix)  # 创建新路径
    return path


def plot_one_box(x, img, color=None, label=None, line_thickness=None, center=None):
    # x: 框的坐标 (x_min, y_min, x_max, y_max)
    # img: 输入图像
    # color: 绘制框的颜色
    # label: 框的标签
    # line_thickness: 框的线条厚度
    tl = line_thickness or round(0.002 * max(img.shape)) + 1  # 线条厚度
    color = color or [random.randint(0, 255) for _ in range(3)]  # 随机颜色
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))  # 框的两个角的坐标
    cv2.rectangle(img, c1, c2, color, thickness=tl,
                  lineType=cv2.LINE_AA)  # 绘制矩形框
    if label:
        tf = max(tl - 1, 1)  # 字体厚度
        t_size = cv2.getTextSize(
            label, 0, fontScale=tl / 3, thickness=tf)[0]  # 字体大小
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3  # 字体右下角的坐标
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # 绘制文字背景
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                    [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)  # 绘制文字
    # 绘制中心点
    if center:
        cv2.circle(img, center, 3, (0, 0, 255),
                   thickness=3, lineType=cv2.LINE_AA)


def load_yolo(weights_path, device='cpu', half=False):
    # Load model
    model = YOLO(weights_path)  # 加载模型

    if half and device != 'cpu':
        model.half()  # Convert to half precision
    return model


def detect_yolo(model, color_image):
    detected_img = color_image.copy()
    src_img = color_image.copy()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    half = False

    # Padded resize
    img_size = 640
    stride = 32
    auto = True
    img = letterbox(src_img, img_size, stride=stride, auto=auto)[0]

    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]

    # Predict
    augment = False
    pred = model(img, augment=augment)[0]

    # NMS
    # conf_thres = 0.7 best1
    # iou_thres = 0.2
    conf_thres = 0.6
    iou_thres = 0.45
    classes = None
    agnostic_nms = False
    max_det = 1000
    pred = non_max_suppression(
        pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

    centers = []
    for i, det in enumerate(pred):
        if len(det):
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], [
                                      src_img.shape[0], src_img.shape[1], 3]).round()

            # Extract and store center coordinates for each detected object along with confidence
            for det_item in det:
                xyxy = det_item[:4]
                conf = det_item[4]
                label = int(det_item[5])
                print(f"xyxy: {xyxy}, conf: {conf},  label: {label}")
                x_min, y_min, x_max, y_max = xyxy
                center_x = int((x_min + x_max) / 2)
                center_y = int((y_min + y_max) / 2)
                centers.append(((center_x, center_y), conf, label))

                # Draw detection box and center point
                plot_one_box(xyxy, detected_img,
                             label=f'{label} {conf:.2f}', color=(0, 255, 0), center=(center_x, center_y))

    # 清理缓存
    del img
    torch.cuda.empty_cache()
    return centers, detected_img


def detect_action():
    # 加载模型
    # model_detector = load_yolo("/home/nx/桌面/test/robot_e/runs/best1.pt")
    model_detector = load_yolo("F:/studing/robot_e/runs/best1.pt")
    try:
        robot = Robot(tcp_host_ip, tcp_port)

        ctx = rs.context()
        print('Waiting for all devices to connect...')
        found = False
        while not found:
            if len(ctx.devices) > 0:
                # 初始化RealSense相机
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.depth, 640,
                                     480, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 640,
                                     480, rs.format.bgr8, 30)
                align_to = rs.stream.color
                align = rs.align(align_to)
                profile = pipeline.start(config)
                print("相机已连接")
                found = True
                robot.move_j_p(photo_ready_waypoint)
                robot.open_gripper(speed=100, force=30)
    except Exception as e:
        print(e)

    capture_trigged = False

    def check_key():
        global capture_trigged
        while True:
            print("press...")
            key_input = input()
            if key_input == ' ':
                capture_trigged = True

    thread = threading.Thread(target=check_key)
    thread.daemon = True
    thread.start()
    # 在你的代码的顶部定义一个全局计数器
    frame_counter = 0
    frame_skip = 5  # 每5帧进行一次预测

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        tmp_depth_img = depth_image.copy()

        # 获取内参
        intrinsics = profile.get_stream(
            rs.stream.color).as_video_stream_profile().get_intrinsics()
        # 每 frame_skip 帧进行一次预测
        if frame_counter % frame_skip == 0:
            try:
                pred_list, detected_img = detect_yolo(
                    model_detector, color_image)
                cv2.imshow("window", detected_img)
                # print("pred_list structure:", pred_list)
            except RuntimeError as e:
                if str(e) == "Frame didn't arrive within 5000":
                    print("Frame didn't arrive within 5000, continue to next frame")
                else:
                    raise e
        frame_counter += 1

        key_input = cv2.waitKey(1)
        if capture_trigged:

            print("空格键被按下")
            target_list = []
            for center, conf, label in pred_list:
                u = int(center[0])
                v = int(center[1])
                tmp_depth = tmp_depth_img[v][u]
                print([u, v], tmp_depth)

                # cam_depth_scale = np.loadtxt(
                #     "F:/studing/jxb/elite_robot (2)/elite_robot/camera_depth_scale_tool.txt",
                #     delimiter=" ",
                # )
                # tmp_z = tmp_depth * cam_depth_scale
                tmp_x, tmp_y, tmp_z = rs.rs2_deproject_pixel_to_point(
                    intrinsics, [u, v], tmp_depth)
                # print("tmp_x1, tmp_y1, tmp_z1:", tmp_x1, tmp_y1, tmp_z1)
                # cam_intrinsics = np.array(
                #     [615.543, 0, 314.882, 0, 616.009, 230.397, 0, 0, 1]
                # ).reshape(3, 3)

                # tmp_x = np.multiply(
                #     u - cam_intrinsics[0][2], tmp_z / cam_intrinsics[0][0])
                # tmp_y = np.multiply(
                #     v - cam_intrinsics[1][2], tmp_z / cam_intrinsics[1][1])
            target_list.append([tmp_x, tmp_y, tmp_z])
            print("相机坐标:", target_list)
            for target_input in target_list:
                pick_action(target_input[0], target_input[1], target_input[2])
                robot.move_j_p(photo_ready_waypoint)
                robot.move_j_p([-0.500, 0, 0.600, -np.pi, 0, -np.pi])
                robot.move_j_p([-0.300, 0, 0.235, -np.pi, 0, -np.pi])
                robot.open_gripper(speed=100, force=30)
                time.sleep(1)
                robot.move_j_p([-0.500, 0, 0.600, -np.pi, 0, -np.pi])
                robot.move_j_p(photo_ready_waypoint)
                capture_trigged = False
            # continue
            break
        time.sleep(0.01)  # 减少主循环中的延迟]
    model_detector.close()
    cv2.destroyAllWindows()


detect_action()
