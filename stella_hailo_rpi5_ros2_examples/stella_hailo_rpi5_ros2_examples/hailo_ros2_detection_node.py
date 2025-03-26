import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo

import time
import threading
import queue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Override to use parse_known_args() instead of parse_args()
from hailo_apps_infra.hailo_rpi_common import get_default_parser as original_get_default_parser
import hailo_apps_infra.hailo_rpi_common as common
def get_default_parser_ros2(*args, **kwargs):
    parser = original_get_default_parser(*args, **kwargs)
    parser.parse_args = lambda: parser.parse_known_args()[0]
    return parser
common.get_default_parser = get_default_parser_ros2

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

from hailo_apps_infra.gstreamer_helper_pipelines import (
    INFERENCE_PIPELINE,
    INFERENCE_PIPELINE_WRAPPER,
    TRACKER_PIPELINE,
    DISPLAY_PIPELINE,
    USER_CALLBACK_PIPELINE,
)
from hailo_apps_infra.gstreamer_app import disable_qos
from stella_hailo_rpi5_ros2_examples.color_config import get_bbox_color

processed_frame_queue = queue.Queue(maxsize=10)

# ------------------------------------------------------------------
# User-defined class to be used in the callback function
# ------------------------------------------------------------------
# Inheritance from the app_callback_class
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42 # New variable example

    def new_function(self): # New function example
        return "The meaning of life is:"

# ------------------------------------------------------------------
# User-defined callback function
# ------------------------------------------------------------------
def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    # Check if the buffer is valid
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Using the user_data to count the number of frames
    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    # Get the caps from the pad
    format, width, height = get_caps_from_pad(pad)
    frame = None
    if format is not None and width is not None and height is not None:
        # Get video frame
        frame = get_numpy_from_buffer(buffer, format, width, height)

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Parse the detections
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()
        color = get_bbox_color(label)
        # cv2 frame for ROS2
        try:
            x1 = round(bbox.xmin() * width)
            y1 = round(bbox.ymin() * height)
            x2 = round(bbox.xmax() * width)
            y2 = round(bbox.ymax() * height)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 1)
            cv2.putText(frame, f"{label}", ((int(x1)+5), (int(y1)+10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        except AttributeError:
            # print("Unable to extract bbox coordinates from HailoBBox; please check API.")
            continue

        if label == "person":
            # Get track ID
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()
            string_to_print += f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
            detection_count += 1
            cv2.putText(frame, f"{track_id}", ((int(x1)+5), (int(y2)-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    if user_data.use_frame:
        # Note: using imshow will not work here, as the callback function is not running in the main thread
        # Let's print the detection count to the frame
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # Example of how to use the new_variable and new_function from the user_data
        # Let's print the new_variable and the result of the new_function to the frame
        cv2.putText(frame, f"{user_data.new_function()} {user_data.new_variable}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # Convert the frame to BGR

    if frame is not None:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        try:
            processed_frame_queue.put_nowait(frame)
        except queue.Full:
            processed_frame_queue.get_nowait()
            processed_frame_queue.put_nowait(frame)

    # print(string_to_print)
    return Gst.PadProbeReturn.OK

# ------------------------------------------------------------------
# ROS2 Gstreamer Application
# ------------------------------------------------------------------
class ROS2DetectionApp(GStreamerDetectionApp):
    def get_pipeline_string(self):
        # source pipeline for image topic
        source_pipeline = (
            "appsrc name=app_source is-live=true format=GST_FORMAT_TIME "
            "! videoconvert ! video/x-raw, format=RGB, width=640, height=480 "
            "! queue name=source_queue ! videoscale "
        )

        detection_pipeline = INFERENCE_PIPELINE(
            hef_path=self.hef_path,
            post_process_so=self.post_process_so,
            batch_size=self.batch_size,
            config_json=self.labels_json,
            additional_params=self.thresholds_str
        )
        detection_pipeline_wrapper = INFERENCE_PIPELINE_WRAPPER(detection_pipeline)
        tracker_pipeline = TRACKER_PIPELINE(class_id=1)
        user_callback_pipeline = USER_CALLBACK_PIPELINE()
        
        # Activate the GStreamer stream display
        # display_pipeline = DISPLAY_PIPELINE(video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps)
        
        # Deactivate the GStreamer stream display
        display_pipeline = DISPLAY_PIPELINE(video_sink="fakesink", sync=self.sync, show_fps=self.show_fps)

        pipeline_string = (
            f"{source_pipeline} ! "
            f"{detection_pipeline_wrapper} ! "
            f"{tracker_pipeline} ! "
            f"{user_callback_pipeline} ! "
            f"{display_pipeline}"
        )
        return pipeline_string

    def shutdown(self, signum=None, frame=None):
        self.pipeline.set_state(Gst.State.NULL)
        GLib.idle_add(self.loop.quit)

    def run(self):
        self.source_type = "ros2"
        Gst.init(None)
        pipeline_string = self.get_pipeline_string()
        try:
            self.pipeline = Gst.parse_launch(pipeline_string)
        except Exception as e:
            exit(1)
        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.bus_call, self.loop)
        if not self.options_menu.disable_callback:
            identity = self.pipeline.get_by_name("identity_callback")
            if identity is not None:
                identity_pad = identity.get_static_pad("src")
                identity_pad.add_probe(Gst.PadProbeType.BUFFER, self.app_callback, self.user_data)
            else:
                print("Warning: identity_callback element not found.")
        else:
            print("Warning: Callback disabled.")
        disable_qos(self.pipeline)
        self.pipeline.set_state(Gst.State.PAUSED)
        new_latency = self.pipeline_latency * Gst.MSECOND
        self.pipeline.set_latency(new_latency)
        self.pipeline.set_state(Gst.State.PLAYING)

# ------------------------------------------------------------------
# ROS2 Node Class
# ------------------------------------------------------------------
class HailoDetectionNode(Node):
    def __init__(self):
        super().__init__("hailo_ros2_detection_node")
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Image, "detection_image", 10)
        self.bridge = CvBridge()
        self.get_logger().info("Detection node started.")

        self.frame_count = 0

        # dummy arguments
        class DummyArgs:
            def __init__(self):
                self.input = ""
                self.use_frame = True
                self.show_fps = False
                self.arch = None
                self.hef_path = None
                self.disable_sync = True
                self.disable_callback = False
                self.dump_dot = False
                self.labels_json = None
        dummy_args = DummyArgs()

        if "TAPPAS_POST_PROC_DIR" not in os.environ:
            os.environ["TAPPAS_POST_PROC_DIR"] = os.getcwd()

        self.user_data = user_app_callback_class()
        self.detection_app = ROS2DetectionApp(app_callback, self.user_data)
        self.detection_app.options_menu = dummy_args
        self.detection_app.video_width = 640
        self.detection_app.video_height = 480
        self.detection_app.video_source = "ros2"
        self.detection_app.run()

        # Set appsrc for image topic
        self.appsrc = self.detection_app.pipeline.get_by_name("app_source")
        if self.appsrc is None:
            self.get_logger().error("appsrc element not found in the pipeline.")
        else:
            self.get_logger().debug("appsrc element found.")
        caps = Gst.Caps.from_string("video/x-raw, format=RGB, width=640, height=480, framerate=30/1")
        self.appsrc.set_property("caps", caps)
        self.get_logger().debug("appsrc caps set to: " + caps.to_string())

        self.create_timer(0.033, self.publish_processed_frame)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        if self.appsrc is None:
            return

        self.frame_count += 1
        self.get_logger().debug(f"[DEBUG] Received frame #{self.frame_count}")

        # OpenCV image to bytes
        data = cv_image.tobytes()
        # Create Gst.Buffer
        buf = Gst.Buffer.new_wrapped(data)
        buf.pts = self.frame_count * Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
        buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
        # Push buffer to appsrc
        ret = self.appsrc.emit("push-buffer", buf)
        if ret == Gst.FlowReturn.OK:
            self.get_logger().debug("Pushed buffer to appsrc successfully.")
        else:
            self.get_logger().debug(f"Failed to push buffer: {ret}")
        
    def publish_processed_frame(self):
        try:
            frame = processed_frame_queue.get_nowait()
        except queue.Empty:
            self.get_logger().debug("[DEBUG] No processed frame available yet.")
            return

        if frame is not None:
            ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_img)
            self.get_logger().debug("[DEBUG] Published processed ROS image.")
        else:
            self.get_logger().debug("[DEBUG] No processed frame available yet.")

def main(args=None):
    rclpy.init(args=args)
    node = HailoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Exception handling to remove unnecessary logs
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                pass
        else:
            pass


if __name__ == "__main__":
    main()