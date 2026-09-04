#!/usr/bin/env python3
"""Measure depth frame-stamp-to-model latency using the deployed camera path."""

from __future__ import annotations

import argparse
import math
import statistics
import time


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _format_stats(values: list[float]) -> str:
    if not values:
        return "n/a"
    return (
        f"p50={statistics.median(values):.2f} ms, "
        f"p95={_percentile(values, 0.95):.2f} ms, max={max(values):.2f} ms"
    )


def _origin_model_depth(depth_rotated):
    """Mirror the deployed origin-camera policy's final depth transform."""
    import numpy as np

    depth = np.flip(np.transpose(depth_rotated, (1, 0)), axis=0)
    depth = np.clip(depth, 0.2, 3.0)
    depth = (depth - 0.2) / 2.8 - 0.5
    depth = np.ascontiguousarray(depth[2:38, :], dtype=np.float32)
    if depth.shape != (36, 36):
        raise ValueError(f"unexpected model depth shape: {depth.shape}")
    return depth


def _self_test() -> None:
    import numpy as np

    values = [1.0, 2.0, 3.0, 4.0]
    assert _percentile(values, 0.5) == 2.5
    assert _format_stats(values).startswith("p50=2.50 ms, p95=3.85 ms")
    depth = _origin_model_depth(np.ones((36, 48), dtype=np.float32))
    assert depth.shape == (36, 36)
    np.testing.assert_allclose(depth, (1.0 - 0.2) / 2.8 - 0.5)
    print("self-test: ok")


def _run(args: argparse.Namespace) -> int:
    from pathlib import Path
    import sys

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    from bxi_example_py_elf3.framework.inference import InferenceRuntime, ModelSpec

    mod_path = (
        Path(__file__).resolve().parents[2]
        / "src/bxi_example_py_elf3/mods/com.bxi.normal_depth"
    )
    if not mod_path.is_dir():
        from ament_index_python.packages import get_package_share_directory

        mod_path = (
            Path(get_package_share_directory("bxi_example_py_elf3"))
            / "mods/com.bxi.normal_depth"
        )
    sys.path.insert(0, str(mod_path))
    from depth_projection import ProjectionSpec, project_depth

    backend = None
    model_inputs = None
    if args.infer:
        if args.model:
            model_path = Path(args.model).expanduser().resolve()
        else:
            from ament_index_python.packages import get_package_share_directory

            model_path = (
                Path(get_package_share_directory("bxi_example_py_elf3"))
                / "mods/com.bxi.normal_depth/assets/dagger2.onnx"
            )
        spec = ModelSpec.portable_onnx(
            model_path,
            input_names=(),
            output_names=(),
        )
        backend = InferenceRuntime().open_backend(spec)
        if tuple(backend.input_names) != ("obs_history", "depth_data"):
            raise ValueError(
                "expected dagger2 inputs ('obs_history', 'depth_data'), got "
                f"{backend.input_names}"
            )
        model_inputs = {
            "obs_history": np.zeros((1, 960), dtype=np.float32),
            "depth_data": np.zeros((1, 36, 36), dtype=np.float32),
        }

    projection = ProjectionSpec(
        output_width=36,
        output_height=48,
        horizontal_fov_deg=45.2,
        vertical_fov_deg=58.0616969,
        minimum_m=0.2,
        maximum_m=3.0,
    )

    class DepthLatencyNode(Node):
        def __init__(self) -> None:
            super().__init__("depth_latency_demo")
            qos = QoSProfile(
                depth=1,
                durability=qos_profile_sensor_data.durability,
                reliability=qos_profile_sensor_data.reliability,
            )
            self.received = 0
            self.processed = 0
            self.processing_errors = 0
            self.camera_info: CameraInfo | None = None
            self.latest_frame = None
            self.last_arrival_ns: int | None = None
            self.last_header_ns: int | None = None
            self.selected_frame = None
            self.last_refresh_ns: int | None = None
            self.last_sampled_frame = 0
            self.policy_samples = 0
            self.reused_samples = 0
            self.arrival_intervals_ms: list[float] = []
            self.header_intervals_ms: list[float] = []
            self.transport_ages_ms: list[float] = []
            self.policy_ages_ms: list[float] = []
            self.receive_to_model_ms: list[float] = []
            self.ready_to_model_ms: list[float] = []
            self.preprocessing_ms: list[float] = []
            self.inference_ms: list[float] = []
            self.publish_to_output_ms: list[float] = []
            self.create_subscription(Image, args.topic, self._on_image, qos)
            self.create_subscription(
                CameraInfo,
                args.camera_info_topic,
                self._on_camera_info,
                qos,
            )
            self.create_timer(args.control_ms / 1000.0, self._sample_policy)

        def _on_camera_info(self, msg: CameraInfo) -> None:
            if (
                msg.width > 0
                and msg.height > 0
                and len(msg.k) == 9
                and msg.k[0] > 0.0
                and msg.k[4] > 0.0
            ):
                self.camera_info = msg

        @staticmethod
        def _depth_to_meters(msg: Image):
            encoding = msg.encoding.lower()
            if encoding in ("16uc1", "mono16"):
                dtype = np.dtype(np.uint16).newbyteorder(
                    ">" if msg.is_bigendian else "<"
                )
                scale = 0.001
            elif encoding == "32fc1":
                dtype = np.dtype(np.float32).newbyteorder(
                    ">" if msg.is_bigendian else "<"
                )
                scale = 1.0
            else:
                raise ValueError(f"unsupported encoding: {msg.encoding}")
            row_values = int(msg.step) // dtype.itemsize
            expected_values = row_values * int(msg.height)
            if msg.width <= 0 or msg.height <= 0 or row_values < msg.width:
                raise ValueError("invalid image dimensions or step")
            if len(msg.data) < expected_values * dtype.itemsize:
                raise ValueError("incomplete image data")
            depth = np.frombuffer(
                msg.data,
                dtype=dtype,
                count=expected_values,
            ).reshape(int(msg.height), row_values)[:, : int(msg.width)]
            return (depth.astype(np.float32) * scale).copy()

        def _on_image(self, msg: Image) -> None:
            arrival_ns = time.monotonic_ns()
            receive_clock_ns = self.get_clock().now().nanoseconds
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            if self.last_arrival_ns is not None:
                self.arrival_intervals_ms.append(
                    (arrival_ns - self.last_arrival_ns) / 1_000_000.0
                )
            if stamp_ns > 0:
                self.transport_ages_ms.append(
                    (receive_clock_ns - stamp_ns) / 1_000_000.0
                )
                if self.last_header_ns is not None:
                    self.header_intervals_ms.append(
                        (stamp_ns - self.last_header_ns) / 1_000_000.0
                    )
                self.last_header_ns = stamp_ns
            self.last_arrival_ns = arrival_ns
            self.received += 1
            if self.camera_info is None:
                return
            try:
                depth_meters = self._depth_to_meters(msg)
                projected, _crop = project_depth(
                    depth_meters,
                    self.camera_info,
                    projection,
                )
                depth_rotated = np.ascontiguousarray(
                    np.rot90(projected, k=-1).astype(np.float32)
                )
                model_depth = _origin_model_depth(depth_rotated)
            except (TypeError, ValueError):
                self.processing_errors += 1
                return
            ready_ns = time.monotonic_ns()
            self.preprocessing_ms.append((ready_ns - arrival_ns) / 1_000_000.0)
            self.processed += 1
            self.latest_frame = (
                self.received,
                stamp_ns if stamp_ns > 0 else receive_clock_ns,
                arrival_ns,
                ready_ns,
                model_depth,
            )

        def _sample_policy(self) -> None:
            if self.latest_frame is None:
                return
            now_mono_ns = time.monotonic_ns()
            update_ns = int(args.policy_update_ms * 1_000_000)
            if (
                update_ns == 0
                or self.last_refresh_ns is None
                or now_mono_ns - self.last_refresh_ns >= update_ns
            ):
                self.selected_frame = self.latest_frame
                self.last_refresh_ns = now_mono_ns
            if self.selected_frame is None:
                return
            frame_id, stamp_ns, arrival_ns, ready_ns, model_depth = (
                self.selected_frame
            )
            model_clock_ns = self.get_clock().now().nanoseconds
            model_mono_ns = time.monotonic_ns()
            self.policy_samples += 1
            if frame_id == self.last_sampled_frame:
                self.reused_samples += 1
            self.last_sampled_frame = frame_id
            publish_to_model_ms = (model_clock_ns - stamp_ns) / 1_000_000.0
            self.policy_ages_ms.append(publish_to_model_ms)
            self.receive_to_model_ms.append(
                (model_mono_ns - arrival_ns) / 1_000_000.0
            )
            self.ready_to_model_ms.append(
                (model_mono_ns - ready_ns) / 1_000_000.0
            )
            if backend is not None and model_inputs is not None:
                np.copyto(model_inputs["depth_data"][0], model_depth)
                inference_started_ns = time.perf_counter_ns()
                outputs = backend.run(model_inputs)
                inference_done_ns = time.perf_counter_ns()
                if not np.isfinite(outputs[backend.output_names[0]]).all():
                    raise RuntimeError("model returned non-finite output")
                self.inference_ms.append(
                    (inference_done_ns - inference_started_ns) / 1_000_000.0
                )
                self.publish_to_output_ms.append(
                    (
                        self.get_clock().now().nanoseconds
                        - stamp_ns
                    )
                    / 1_000_000.0
                )

    rclpy.init()
    node = DepthLatencyNode()
    started = time.monotonic()
    print(
        f"topic={args.topic}, camera_info={args.camera_info_topic}, "
        f"duration={args.duration:.1f}s, "
        f"control={args.control_ms:.1f}ms, policy_image_update={args.policy_update_ms:.1f}ms"
    )
    print(
        "mode=origin_camera projection + 36x36 model input"
        + (f" + {backend.backend_name} inference" if backend is not None else "")
    )
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    elapsed = max(time.monotonic() - started, 1e-9)
    print(f"frames={node.received}, receive_rate={node.received / elapsed:.2f} fps")
    print(
        f"model_inputs={node.processed}, "
        f"input_rate={node.processed / elapsed:.2f} fps, "
        f"processing_errors={node.processing_errors}"
    )
    print(f"arrival interval:       {_format_stats(node.arrival_intervals_ms)}")
    print(f"header interval:        {_format_stats(node.header_intervals_ms)}")
    print(f"frame-stamp-to-receive: {_format_stats(node.transport_ages_ms)}")
    print(f"receive preprocessing:  {_format_stats(node.preprocessing_ms)}")
    print(f"receive-to-model input: {_format_stats(node.receive_to_model_ms)}")
    print(f"ready-to-model input:   {_format_stats(node.ready_to_model_ms)}")
    print(f"frame-stamp-to-model:   {_format_stats(node.policy_ages_ms)}")
    if backend is not None:
        print(f"model inference:        {_format_stats(node.inference_ms)}")
        print(f"frame-stamp-to-output:  {_format_stats(node.publish_to_output_ms)}")
    if node.policy_samples:
        print(
            "policy frame reuse:     "
            f"{node.reused_samples}/{node.policy_samples} "
            f"({100.0 * node.reused_samples / node.policy_samples:.1f}%)"
        )

    if node.received == 0:
        print("result: FAIL - no depth frames received")
        result = 2
    elif node.processed == 0:
        print("result: FAIL - no valid depth frame reached the model input path")
        result = 2
    elif node.transport_ages_ms and min(node.transport_ages_ms) < -5.0:
        print("result: INVALID - publisher and subscriber clocks do not match")
        result = 2
    elif not node.transport_ages_ms:
        print("result: PARTIAL - header stamps are missing; transport age was not measured")
        result = 0
    else:
        transport_p95 = (
            _percentile(node.transport_ages_ms, 0.95)
            if node.transport_ages_ms
            else 0.0
        )
        policy_p95 = _percentile(node.policy_ages_ms, 0.95)
        if transport_p95 > 50.0:
            print("result: DELAY - frame-stamp-to-receive age is high (p95 > 50 ms)")
        elif policy_p95 > 100.0:
            print("result: DELAY - model input often uses depth older than 100 ms")
        else:
            print("result: OK - no large frame-stamp-to-model delay detected")
        result = 0

    print(
        "note: the Orbbec driver prefers global capture timestamps and falls back "
        "to host-receive/publish time when unavailable; check the camera log."
    )
    node.destroy_node()
    rclpy.shutdown()
    if backend is not None:
        backend.close()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/hardware/head_depth_camera/depth/image_rect_raw",
        help="depth Image topic",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/hardware/head_depth_camera/depth/camera_info",
        help="depth CameraInfo topic",
    )
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--control-ms", type=float, default=20.0)
    parser.add_argument("--policy-update-ms", type=float, default=0.0)
    parser.add_argument(
        "--infer",
        action="store_true",
        help="also run dagger2 inference; use only while the robot controller is stopped",
    )
    parser.add_argument(
        "--model",
        help="dagger2 ONNX path; defaults to the installed bxi_example_py_elf3 asset",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        _self_test()
        return 0
    if args.duration <= 0 or args.control_ms <= 0 or args.policy_update_ms < 0:
        parser.error(
            "duration/control period must be positive; update period cannot be negative"
        )
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main())
