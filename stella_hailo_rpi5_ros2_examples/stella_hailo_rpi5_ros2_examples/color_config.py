# color_config.py

YOLO_COLOR_MAP = {
    "person": (255, 0, 0),
    "bicycle": (0, 255, 0),
    "car": (0, 0, 255),
    "motorcycle": (255, 255, 0),
    "airplane": (0, 255, 255),
    "bus": (255, 0, 255),
    "train": (128, 0, 0),
    "truck": (0, 128, 0),
    "boat": (0, 0, 128),
    "traffic light": (128, 128, 0),
    "fire hydrant": (128, 0, 128),
    "stop sign": (0, 128, 128),
    "parking meter": (255, 128, 0),
    "bench": (255, 0, 128),
    "bird": (128, 255, 0),
    "cat": (0, 255, 128),
    "dog": (128, 128, 255),
    "horse": (255, 128, 128),
    "sheep": (128, 255, 128),
    "cow": (192, 0, 0),
    "elephant": (0, 192, 0),
    "bear": (0, 0, 192),
    "zebra": (192, 192, 0),
    "giraffe": (192, 0, 192),
    "backpack": (0, 192, 192),
    "umbrella": (255, 192, 0),
    "handbag": (255, 0, 192),
    "tie": (192, 255, 0),
    "suitcase": (0, 255, 192),
    "frisbee": (192, 192, 255),
    "skis": (255, 192, 192),
    "snowboard": (192, 255, 192),
    "sports ball": (64, 0, 0),
    "kite": (0, 64, 0),
    "baseball bat": (0, 0, 64),
    "baseball glove": (64, 64, 0),
    "skateboard": (64, 0, 64),
    "surfboard": (0, 64, 64),
    "tennis racket": (255, 64, 0),
    "bottle": (255, 0, 64),
    "wine glass": (64, 255, 0),
    "cup": (0, 255, 64),
    "fork": (64, 64, 255),
    "knife": (255, 64, 64),
    "spoon": (64, 255, 64),
    "bowl": (160, 0, 0),
    "banana": (0, 160, 0),
    "apple": (0, 0, 160),
    "sandwich": (160, 160, 0),
    "orange": (160, 0, 160),
    "broccoli": (0, 160, 160),
    "carrot": (255, 160, 0),
    "hot dog": (255, 0, 160),
    "pizza": (160, 255, 0),
    "donut": (0, 255, 160),
    "cake": (160, 160, 255),
    "chair": (255, 160, 160),
    "couch": (160, 255, 160),
    "potted plant": (96, 0, 0),
    "bed": (0, 96, 0),
    "dining table": (0, 0, 96),
    "toilet": (96, 96, 0),
    "tv": (96, 0, 96),
    "laptop": (0, 96, 96),
    "mouse": (255, 96, 0),
    "remote": (255, 0, 96),
    "keyboard": (96, 255, 0),
    "cell phone": (0, 255, 96),
    "microwave": (96, 96, 255),
    "oven": (255, 96, 96),
    "toaster": (96, 255, 96),
    "sink": (224, 0, 0),
    "refrigerator": (0, 224, 0),
    "book": (0, 0, 224),
    "clock": (224, 224, 0),
    "vase": (224, 0, 224),
    "scissors": (0, 224, 224),
    "teddy bear": (255, 224, 0),
    "hair drier": (255, 0, 224),
    "toothbrush": (224, 255, 0),
}

def get_bbox_color(label):
    return YOLO_COLOR_MAP.get(label.lower(), (255, 255, 255))
