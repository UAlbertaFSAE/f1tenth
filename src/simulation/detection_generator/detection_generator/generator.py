# MIT License

# Copyright (c) 2026 Krupal Shah

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import csv
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rc_interfaces.msg import Cone, Cones
from rclpy.node import Node


@dataclass
class ConeFrame:
    """Container for one published frame and its cone detections."""

    frame_id: int
    cones: Cones


class DetectionGenerator(Node):
    """Publish cone detections frame-by-frame from a CSV file."""

    CSV_FIELD_COUNT = 4

    def __init__(self) -> None:
        """Initialize parameters, publisher, timer, and CSV-backed frame cache."""
        super().__init__("detection_generator_node")

        self.declare_parameter("track_type", "straight")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("cone_topic", "/cone_data")
        self.declare_parameter("loop_track", True)
        self.declare_parameter("frame_rate_hz", 5.0)
        self.declare_parameter("qos_depth", 10)

        track_type = self.get_parameter(
            "track_type").get_parameter_value().string_value
        configured_csv_path = self.get_parameter(
            "csv_path").get_parameter_value().string_value
        cone_topic = self.get_parameter(
            "cone_topic").get_parameter_value().string_value
        self.loop_track = self.get_parameter(
            "loop_track").get_parameter_value().bool_value
        frame_rate_hz = self.get_parameter(
            "frame_rate_hz").get_parameter_value().double_value
        qos_depth = self.get_parameter(
            "qos_depth").get_parameter_value().integer_value

        if frame_rate_hz <= 0.0:
            self.get_logger().warn("frame_rate_hz must be > 0. Falling back to 5.0")
            frame_rate_hz = 5.0
        if qos_depth < 1:
            self.get_logger().warn("qos_depth must be >= 1. Falling back to 10")
            qos_depth = 10

        csv_path = self.resolve_csv_path(configured_csv_path, track_type)
        self.frames = self.read_csv(csv_path)
        self.next_frame_index = 0

        self.cone_publisher = self.create_publisher(
            Cones, cone_topic, int(qos_depth))
        self.publish_timer = self.create_timer(
            1.0 / frame_rate_hz, self.publish_next_frame
        )

        total_cones = sum(len(frame.cones.cones) for frame in self.frames)
        self.get_logger().info(
            f"Loaded {len(self.frames)} frames ({total_cones} cones) from {csv_path}"
        )
        self.get_logger().info(
            f"Publishing one cone frame at {frame_rate_hz:.2f} Hz on {cone_topic}"
        )

    def publish_next_frame(self) -> None:
        """Publish the next frame and handle looping or one-shot mode."""
        if not self.frames:
            return

        if self.next_frame_index >= len(self.frames):
            if not self.loop_track:
                self.get_logger().info("Finished publishing all cone frames once")
                return
            self.next_frame_index = 0

        frame = self.frames[self.next_frame_index]
        self.cone_publisher.publish(frame.cones)
        self.get_logger().debug(
            f"Published frame_id={frame.frame_id} with {len(frame.cones.cones)} cones"
        )
        self.next_frame_index += 1

    def resolve_csv_path(self, configured_path: str, track_type: str) -> str:
        """Resolve configured or default track CSV path from package share."""
        share_dir = Path(get_package_share_directory("detection_generator"))

        if configured_path:
            configured = Path(configured_path)
            if configured.is_absolute():
                return str(configured)
            return str(share_dir / configured)

        normalized_track = track_type.strip().lower()
        if normalized_track not in {"straight", "eight", "curved"}:
            self.get_logger().warn(
                "Unsupported track_type '%s'. Falling back to 'straight'. "
                "Valid values are: straight, eight, curved.",
                track_type,
            )
            normalized_track = "straight"

        return str(share_dir / "data" / f"{normalized_track}.csv")

    def read_csv(self, path: str) -> list[ConeFrame]:
        """Read track CSV rows and group cones by frame id."""
        csv_path = Path(path)
        if not csv_path.exists() or not csv_path.is_file():
            self.get_logger().error(f"Failed to open track csv: {path}")
            return []

        self.get_logger().info(f"Opened track csv: {path}")

        frames: list[ConeFrame] = []
        frame_index_by_id: dict[int, int] = {}

        with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                if not row:
                    continue

                if len(row) != self.CSV_FIELD_COUNT:
                    self.get_logger().warn(
                        "Skipping malformed row in %s. Expected %d fields, got %d: %s",
                        path,
                        self.CSV_FIELD_COUNT,
                        len(row),
                        ",".join(row),
                    )
                    continue

                frame_text = row[0].strip()
                if not frame_text:
                    continue

                if not (frame_text[0].isdigit() or frame_text[0] == "-"):
                    continue

                try:
                    frame_id = int(frame_text)
                    x = float(row[1].strip())
                    y = float(row[2].strip())
                    color = row[3].strip()
                except ValueError:
                    self.get_logger().warn(
                        "Skipping non-numeric row in %s: %s", path, ",".join(
                            row)
                    )
                    continue

                if not color:
                    self.get_logger().warn(
                        "Skipping row with empty cone color in %s: %s",
                        path,
                        ",".join(row),
                    )
                    continue

                cone = Cone()
                cone.x = x
                cone.y = y
                cone.color = color

                if frame_id not in frame_index_by_id:
                    frame = ConeFrame(frame_id=frame_id, cones=Cones())
                    frames.append(frame)
                    frame_index_by_id[frame_id] = len(frames) - 1

                frames[frame_index_by_id[frame_id]].cones.cones.append(cone)

        return frames


def main(args=None) -> None:
    """Run the detections generator ROS2 node."""
    rclpy.init(args=args)
    node = DetectionGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
