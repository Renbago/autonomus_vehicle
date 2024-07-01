import torch
import cv2
import random
import time
import pathlib
from ultralytics import YOLO

import modules.utils as utils
from modules.autobackend import AutoBackend


def tensorrt_detection(model, source, image):
    # Preprocess
    im = utils.preprocess(image)

    # Inference
    preds = model(im)

    # Post Process
    results = utils.postprocess(preds, im, image, model.names, source)
    d = results[0].boxes

    # Get information from result
    tensor_size = d.cls.size()[0]
    if (tensor_size > 1):
        cls, conf, box = d.cls.squeeze(), d.conf.squeeze(), d.xyxy.squeeze()
    else:
        cls, conf, box = d.cls, d.conf, d.xyxy

    return cls, conf, box


def yolov8_detection(model, image):
    # Update object localizer
    results = model.predict(image, imgsz=640, conf=0.5, verbose=False)
    result = results[0].cpu()

    # Get information from result
    box = result.boxes.xyxy.numpy()
    conf = result.boxes.conf.numpy()
    cls = result.boxes.cls.numpy().astype(int)

    return cls, conf, box


def detection(model_path, source, name):
    # Check File Extension
    file_extension = pathlib.Path(model_path).suffix

    if (file_extension == ".engine"):
        model = AutoBackend(model_path, device=torch.device('cuda:0'), fp16=True)
        # Warmup
        model.warmup()
    else:
        model = YOLO(model_path)

    # Class Name and Colors
    label_map = model.names
    COLORS = [[random.randint(0, 255) for _ in range(3)] for _ in label_map]

    # FPS Detection
    frame_count = 0
    total_fps = 0
    avg_fps = 0

    # FPS Video
    video_cap = cv2.VideoCapture(source)

    total_frames = int(video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_width = int(video_cap.get(3))
    frame_height = int(video_cap.get(4))

    video_frames = []

    while video_cap.isOpened():
        ret, frame = video_cap.read()
        if not ret:
            break

        # # Start Time
        start = time.time()

        # Detection
        if (file_extension == ".engine"):
            cls, conf, box = tensorrt_detection(model, source, frame)
        else:
            cls, conf, box = yolov8_detection(model, frame)

        # Pack together for easy use
        detection_output = list(zip(cls, conf, box))
        image_output = utils.draw_box(frame, detection_output, label_map, COLORS)

        end = time.time()
        # # End Time

        # Draw FPS
        frame_count += 1
        fps = 1 / (end - start)
        total_fps = total_fps + fps
        avg_fps = total_fps / frame_count

        image_output = utils.draw_fps(avg_fps, image_output)

        # Append frame to array
        video_frames.append(image_output)

        #
        print("(%2d / %2d) Frames Processed" % (frame_count, total_frames))

    print(avg_fps)

    # Get a file name
    file_name = utils.get_name(source)
    # Get Save Path
    folder_name = name
    save_path = utils.get_save_path(file_name, folder_name)
    # Create VideoWriter object.
    out = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'XVID'), int(avg_fps), (frame_width, frame_height))

    for frame in video_frames:
        out.write(frame)

    out.release()

    print("Video is saved in: " + save_path)


detection("besLevhaS2.engine", "video.mp4", "detection")