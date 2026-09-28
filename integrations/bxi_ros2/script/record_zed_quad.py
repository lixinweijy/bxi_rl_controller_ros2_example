#!/usr/bin/python3
"""Record four ZED cameras as an FPS-labelled 2x2 H.264 MP4."""

import argparse
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import gi
import numpy as np
import pyzed.sl as sl

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--tile-width", type=int, default=640)
    parser.add_argument("--tile-height", type=int, default=400)
    parser.add_argument("--bitrate", type=int, default=12_000_000)
    parser.add_argument("--serials", help="four comma-separated ZED serial numbers")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def open_cameras(serials, fps):
    cameras = []
    for serial in serials:
        init = sl.InitParameters()
        init.set_from_serial_number(serial)
        init.camera_resolution = sl.RESOLUTION.HD1200
        init.camera_fps = fps
        init.depth_mode = sl.DEPTH_MODE.NONE
        camera = sl.Camera()
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            for opened in cameras:
                opened.close()
            raise RuntimeError(f"cannot open ZED {serial}: {status}")
        cameras.append(camera)
    return cameras


def make_pipeline(width, height, fps, bitrate, output, preview):
    overlays = " ".join(
        f"textoverlay name=label{i} halignment={horizontal} valignment={vertical} "
        'font-desc="Sans Bold 18" shaded-background=true !'
        for i, (horizontal, vertical) in enumerate(
            (("left", "top"), ("right", "top"), ("left", "bottom"), ("right", "bottom"))
        )
    )
    pipeline_text = (
        f'appsrc name=src is-live=true do-timestamp=true format=time '
        f'caps="video/x-raw,format=BGRA,width={width * 2},height={height * 2},framerate={fps}/1" '
        f"! queue max-size-buffers=2 leaky=downstream ! {overlays} "
        "tee name=t "
        "t. ! queue ! videoconvert ! video/x-raw,format=I420 "
        "! nvvideoconvert ! video/x-raw(memory:NVMM),format=NV12 "
        f"! nvv4l2h264enc bitrate={bitrate} insert-sps-pps=true "
        "! h264parse ! mp4mux ! filesink name=output "
        + (
            "t. ! queue ! videoconvert ! autovideosink sync=false "
            if preview
            else "t. ! queue ! fakesink sync=false "
        )
    )
    pipeline = Gst.parse_launch(pipeline_text)
    pipeline.get_by_name("output").set_property("location", str(output))
    return pipeline


def capture(camera, resolution, stop, frames, lock, counts, index, errors):
    runtime = sl.RuntimeParameters()
    image = sl.Mat()
    while not stop.is_set():
        status = camera.grab(runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            errors[index] += 1
            time.sleep(0.005)
            continue
        camera.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, resolution)
        frame = np.array(image.get_data(), copy=True)
        with lock:
            frames[index] = frame
        counts[index] += 1


def main():
    args = arguments()
    if args.duration <= 0 or args.fps <= 0 or args.tile_width <= 0 or args.tile_height <= 0:
        raise SystemExit("duration, fps and tile dimensions must be positive")

    devices = sl.Camera.get_device_list()
    available = [int(device.serial_number) for device in devices]
    serials = (
        [int(value) for value in args.serials.split(",")]
        if args.serials
        else available[:4]
    )
    if len(serials) != 4 or any(serial not in available for serial in serials):
        raise SystemExit(f"need four available ZED serials; available={available}, requested={serials}")

    output = args.output or Path.home() / "Videos" / (
        f"zed_quad_{datetime.now():%Y%m%d_%H%M%S}.mp4"
    )
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    Gst.init(None)
    cameras = open_cameras(serials, args.fps)
    pipeline = make_pipeline(
        args.tile_width,
        args.tile_height,
        args.fps,
        args.bitrate,
        output,
        args.preview,
    )
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    counts = [0] * 4
    errors = [0] * 4
    frames = [None] * 4
    frame_lock = threading.Lock()
    resolution = sl.Resolution(args.tile_width, args.tile_height)
    threads = [
        threading.Thread(
            target=capture,
            args=(
                cameras[i],
                resolution,
                stop,
                frames,
                frame_lock,
                counts,
                i,
                errors,
            ),
            daemon=True,
        )
        for i in range(4)
    ]

    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)
    for thread in threads:
        thread.start()
    started = time.monotonic()
    previous_time = started
    previous_counts = counts.copy()
    next_frame = started
    next_report = started + 1.0
    source = pipeline.get_by_name("src")
    try:
        while not stop.is_set():
            now = time.monotonic()
            if now - started >= args.duration:
                break
            if now < next_frame:
                stop.wait(next_frame - now)
                continue
            next_frame += 1.0 / args.fps
            with frame_lock:
                current = list(frames)
            if all(frame is not None for frame in current):
                composite = np.ascontiguousarray(
                    np.concatenate(
                        (
                            np.concatenate((current[0], current[1]), axis=1),
                            np.concatenate((current[2], current[3]), axis=1),
                        ),
                        axis=0,
                    )
                )
                buffer = Gst.Buffer.new_allocate(None, composite.nbytes, None)
                buffer.fill(0, composite.tobytes())
                if source.emit("push-buffer", buffer) != Gst.FlowReturn.OK:
                    raise RuntimeError("GStreamer stopped accepting video frames")
            if now >= next_report:
                rates = [
                    (counts[i] - previous_counts[i]) / (now - previous_time)
                    for i in range(4)
                ]
                for i, rate in enumerate(rates):
                    pipeline.get_by_name(f"label{i}").set_property(
                        "text", f"ZED {serials[i]}  {rate:5.1f} FPS"
                    )
                print(
                    f"{now - started:6.1f}s  "
                    + "  ".join(f"{serials[i]}={rates[i]:5.1f} FPS" for i in range(4)),
                    flush=True,
                )
                previous_time = now
                previous_counts = counts.copy()
                next_report = now + 1.0
            message = bus.pop_filtered(Gst.MessageType.ERROR)
            if message:
                error, debug = message.parse_error()
                raise RuntimeError(f"GStreamer error: {error}; {debug}")
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        source.emit("end-of-stream")
        bus.timed_pop_filtered(10 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        pipeline.set_state(Gst.State.NULL)
        for camera in cameras:
            camera.close()

    elapsed = time.monotonic() - started
    print(f"saved {output} ({elapsed:.1f}s), frames={counts}, grab_errors={errors}")
    return 0 if output.is_file() and output.stat().st_size > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
