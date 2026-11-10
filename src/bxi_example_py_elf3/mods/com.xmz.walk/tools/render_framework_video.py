#!/usr/bin/env python3
"""Render the saved framework simulation, without inference or hardware calls.

Usage: render_framework_video.py ELF4_DIR TRACE_DIR OUTPUT_MP4
Requires the local Noto CJK font, imageio/ffmpeg and MuJoCo OSMesa renderer.
"""
import json
import os
from pathlib import Path
import sys

os.environ["MUJOCO_GL"] = "osmesa"
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def main():
    elf4, traces, output = map(Path, sys.argv[1:])
    config = json.loads((elf4 / "config.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(elf4 / "model/elf4.xml"))
    model.vis.quality.offsamples = 0
    summaries = {result["case"]: result for result in json.loads((traces / "framework_simulation.json").read_text())}
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in config["joint_order"]]
    assert len(ids) == 29 and min(ids) >= 0
    qadr = model.jnt_qposadr[ids]
    data = mujoco.MjData(model)
    recordings = []
    for case in ("forward", "turn"):
        with np.load(traces / f"framework_{case}.npz", allow_pickle=False) as source:
            record = {k: source[k].copy() for k in ("q", "base_position", "base_quat", "command", "state")}
        count = len(record["q"])
        for key, width in (("q", 29), ("base_position", 3), ("base_quat", 4), ("command", 3)):
            assert record[key].shape == (count, width) and np.isfinite(record[key]).all(), key
        assert record["state"].shape == (count,)
        walk = np.flatnonzero(record["state"] == "com.xmz.walk/walk")
        assert len(walk) >= 1000
        record["walk_start"] = int(walk[0])
        # The old state remains WALK while the exit transition restores the hoist.
        stop = record["walk_start"] + round(summaries[case]["unsupported_walk_seconds"] / .02)
        assert np.all(record["state"][record["walk_start"]:stop] == "com.xmz.walk/walk")
        record["walk_end"] = stop - 1
        recordings.append(record)
    # End before the exit test restores the hoist and resets the root position.
    steps = min(record["walk_end"] + 1 for record in recordings)
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(font_path, 22)
    small = ImageFont.truetype(font_path, 17)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance, camera.elevation, camera.azimuth = 3.3, -15, 135
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0  # Hide the diagnostic collision solids, keep visual meshes.
    output.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with mujoco.Renderer(model, width=640, height=560) as renderer, imageio.get_writer(
            output, fps=25, codec="libx264", quality=8, ffmpeg_params=["-movflags", "+faststart"]) as writer:
        for step in range(0, steps, 2):
            frame = Image.new("RGB", (1280, 720), "#17202a")
            draw = ImageDraw.Draw(frame)
            for panel, (record, title) in enumerate(zip(recordings, ("直行：vx = 0.5 m/s", "转向：yaw = 0.5 rad/s"))):
                left = panel * 640
                data.qpos[:3] = record["base_position"][step]
                data.qpos[3:7] = record["base_quat"][step]
                data.qpos[qadr] = record["q"][step]
                mujoco.mj_forward(model, data)
                # Fixed world azimuth makes the observed heading drift visible.
                camera.lookat[:] = data.qpos[:3] + [0, 0, -.5]
                renderer.update_scene(data, camera=camera, scene_option=options)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                frame.paste(Image.fromarray(renderer.render()), (left, 76))
                supported = step < record["walk_start"]
                status = "PD / 过渡：模拟吊挂支撑" if supported else "WALK：已释放支撑"
                draw.text((left+18, 7), title, font=font, fill="white")
                draw.text((left+18, 43), f"t = {(step+1)*.02:05.2f} s    {status}", font=small, fill="#9ad9cc")
                # Both maps use the same scale and world axes; an arrow shows heading.
                origin = np.array([left+94, 165.])
                draw.rectangle((left+10, 84, left+178, 254), fill="#17202a", outline="#728392")
                for axis in ((left+20, 165, left+168, 165), (left+94, 94, left+94, 244)):
                    draw.line(axis, fill="#455360")
                xy = record["base_position"][:step+1:2, :2]
                points = origin + xy * [30, -30]
                if len(points) > 1:
                    draw.line([tuple(v) for v in points], fill="#e5a05d", width=2)
                current = origin + record["base_position"][step, :2] * [30, -30]
                q = record["base_quat"][step]
                yaw = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
                tip = current + 12*np.array([np.cos(yaw), -np.sin(yaw)])
                draw.ellipse((*tuple(current-3), *tuple(current+3)), fill="white")
                draw.line((*current, *tip), fill="#9ad9cc", width=3)
                draw.text((left+17, 258), "轨迹：30 px = 1 m / +X→ +Y↑", font=small, fill="#d5dce2")
            draw.line((640, 0, 640, 640), fill="#728392", width=2)
            draw.text((20, 648), "新官方框架仿真数据回放 · MuJoCo / ELF4 · 1×速度 · 未连接真机", font=font, fill="white")
            draw.text((20, 686), "直行仍有明显偏航；画面使用 ELF4 几何，真实凯旋双臂干涉尚未验证。", font=small, fill="#efb886")
            writer.append_data(np.asarray(frame))
            written += 1
            if written % 125 == 0:
                print(f"rendered {written/25:.0f}/{steps*.02:.0f} seconds", flush=True)
            if step == 750:
                frame.save(output.with_suffix(".png"))
    assert written == len(range(0, steps, 2)) and output.stat().st_size > 10000
    print(f"Saved {output}: {written} frames, {written/25:.2f} s", flush=True)


if __name__ == "__main__":
    main()
