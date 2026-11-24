#!/usr/bin/env python3
"""Collect live G2 telemetry directly into a local LeRobot dataset (no upload)."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import shlex
import struct
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
CAMERAS = ["head_color", "hand_left_color", "hand_right_color", "head_stereo_left",
           "head_stereo_right", "head_left_fisheye", "head_right_fisheye", "head_back_fisheye"]


def read_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        data = stream.read(size - len(result))
        if not data:
            raise EOFError("GDK source disconnected; see source logs")
        result.extend(data)
    return bytes(result)


def read_packet(stream):
    length = struct.unpack("!I", read_exact(stream, 4))[0]
    if not 0 < length <= 1_000_000:
        raise ValueError("Invalid protocol header (SDK stdout contamination?)")
    metadata = json.loads(read_exact(stream, length))
    sizes = metadata.get("payload_sizes", [])
    if any(not isinstance(n, int) or n < 0 for n in sizes) or sum(sizes) > 128_000_000:
        raise ValueError("Invalid image payload size")
    return metadata, [read_exact(stream, n) for n in sizes]


class Source:
    def __init__(self, args):
        options = ["--fps", str(args.fps), "--timeout", str(args.timeout),
                   "--max-sync-ms", str(args.max_sync_ms), "--max-action-age-ms", str(args.max_action_age_ms),
                   "--max-width", str(args.max_width), "--transport", args.transport,
                   "--jpeg-quality", str(args.jpeg_quality), "--state-topic", args.state_topic,
                   "--action-topic", args.action_topic, "--cameras", *args.cameras]
        if args.simulate:
            options += ["--simulate"]
            command = [sys.executable, "-u", str(HERE / "gdk_source.py"), *options]
        elif args.local_gdk:
            command = [args.gdk_python, "-u", str(HERE / "gdk_source.py"), *options]
        else:
            # Stream code in the command, leaving stdin free for START/STOP commands.
            # OpenSSH asks for passwords via the terminal; no credentials are saved here.
            code = (HERE / "gdk_source.py").read_text(encoding="utf-8")
            remote = "source " + shlex.quote(args.remote_env) + " >/dev/null 2>&1 && exec python3 -u -c "
            remote += shlex.quote(code) + " " + shlex.join(options)
            command = ["ssh", "-T", "-o", "Compression=no", "-o", "ConnectTimeout=8",
                       "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3"]
            if args.ssh_control_dir:
                control_path = str(Path(args.ssh_control_dir).expanduser() / "%C")
                if args.require_ssh_control:
                    command += ["-o", "BatchMode=yes", "-o", "ControlMaster=no", "-o", "ControlPath=" + control_path]
                else:
                    command += ["-o", "ControlMaster=auto", "-o", "ControlPersist=yes", "-o", "ControlPath=" + control_path]
            if args.known_hosts:
                command += ["-o", "UserKnownHostsFile=" + str(Path(args.known_hosts).expanduser()),
                            "-o", "StrictHostKeyChecking=yes"]
            command += [args.ssh, "bash -lc " + shlex.quote(remote)]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.messages = queue.Queue(maxsize=4)
        self.draining = threading.Event()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            while True:
                packet = read_packet(self.process.stdout)
                if not self.draining.is_set():
                    self.messages.put(packet, timeout=5)
                if packet[0]["kind"] in ("end", "error"):
                    return
        except Exception as exc:
            if not self.draining.is_set():
                try:
                    self.messages.put(({"kind": "error", "message": str(exc)}, []), timeout=1)
                except queue.Full:
                    pass

    def receive(self, timeout):
        try:
            metadata, payloads = self.messages.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError("Timed out waiting for GDK data; check source logs/network") from None
        if metadata["kind"] == "error":
            raise RuntimeError(metadata["message"])
        return metadata, payloads

    def start(self):
        self.process.stdin.write(b"START\n")
        self.process.stdin.flush()

    def close(self):
        self.draining.set()
        try:
            self.process.stdin.write(b"STOP\n")
            self.process.stdin.flush()
        except (OSError, ValueError):
            pass
        # Drain queued packets so the reader can consume remaining pipe bytes.
        while not self.messages.empty():
            try:
                self.messages.get_nowait()
            except queue.Empty:
                break
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdin.close()
        self.process.stdout.close()


def decode_images(metadata, payloads, ready):
    import cv2
    import numpy as np
    if len(payloads) != len(ready["cameras"]):
        raise ValueError("Wrong camera count")
    result = {}
    for name, shape, data in zip(ready["cameras"], ready["shapes"], payloads):
        if ready["transport"] == "raw":
            image = np.frombuffer(data, np.uint8).reshape(shape)
        else:
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("Invalid image: " + name)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if list(image.shape) != shape:
            raise ValueError("Unexpected image shape: " + name)
        result["observation.images." + name] = image
    return result


def feature_schema(ready):
    names = ready["joint_names"]
    features = {key: {"dtype": "float32", "shape": (len(names),), "names": names}
                for key in ("observation.state", "action")}
    grippers = ready.get("gripper_names", [])
    if grippers:
        if len(grippers) != 2:
            raise ValueError("Expected left and right gripper names")
        features["observation.gripper"] = {"dtype": "float32", "shape": (2,), "names": grippers}
        for side in ("left", "right"):
            features["gripper_" + side + "_timestamp_ns"] = {"dtype": "int64", "shape": (1,), "names": None}
    for key in ("source_timestamp_ns", "state_timestamp_ns", "action_timestamp_ns"):
        features[key] = {"dtype": "int64", "shape": (1,), "names": None}
    for name, shape in zip(ready["cameras"], ready["shapes"]):
        features["observation.images." + name] = {"dtype": "video", "shape": tuple(shape),
                                                   "names": ["height", "width", "channels"]}
        features["camera_timestamp_ns." + name] = {"dtype": "int64", "shape": (1,), "names": None}
    return features


def make_frame(metadata, payloads, ready, args, expected_index, previous):
    import numpy as np
    if metadata.get("kind") != "frame" or metadata["index"] != expected_index:
        raise ValueError("Missing or out-of-order frame; episode interrupted")
    if len(metadata["camera_ns"]) != len(ready["cameras"]):
        raise ValueError("Missing camera timestamp")
    t = metadata["target_ns"]
    if previous is not None and abs(t - previous - round(1e9 / args.fps)) > 2:
        raise ValueError("Nonuniform sampling timeline")
    if abs(metadata["state_ns"] - t) > args.max_sync_ms * 1e6:
        raise ValueError("State synchronization error")
    if not 0 <= t - metadata["action_ns"] <= args.max_action_age_ms * 1e6:
        raise ValueError("Action is noncausal or stale")
    if any(abs(stamp - t) > args.max_sync_ms * 1e6 for stamp in metadata["camera_ns"]):
        raise ValueError("Camera synchronization error")
    frame = decode_images(metadata, payloads, ready)
    if ready.get("gripper_names"):
        values = np.asarray(metadata.get("gripper", []), dtype=np.float32)
        stamps = metadata.get("gripper_ns", [])
        if values.shape != (2,) or not np.isfinite(values).all() or len(stamps) != 2:
            raise ValueError("Invalid left/right gripper data")
        if any(stamp <= 0 or abs(stamp - t) > args.max_sync_ms * 1e6 for stamp in stamps):
            raise ValueError("Gripper synchronization error")
        frame["observation.gripper"] = values
        for side, stamp in zip(("left", "right"), stamps):
            frame["gripper_" + side + "_timestamp_ns"] = np.array([stamp], dtype=np.int64)
    for key, source_key in (("observation.state", "state"), ("action", "action")):
        value = np.asarray(metadata[source_key], dtype=np.float32)
        if value.shape != (len(ready["joint_names"]),) or not np.isfinite(value).all():
            raise ValueError("Invalid " + key)
        frame[key] = value
    for key, source_key in (("source_timestamp_ns", "target_ns"), ("state_timestamp_ns", "state_ns"),
                             ("action_timestamp_ns", "action_ns")):
        frame[key] = np.array([metadata[source_key]], dtype=np.int64)
    for name, stamp in zip(ready["cameras"], metadata["camera_ns"]):
        frame["camera_timestamp_ns." + name] = np.array([stamp], dtype=np.int64)
    frame["task"] = ("SYNTHETIC TEST: " if args.simulate else "") + args.task
    return frame


def ensure_writer_capacity(dataset):
    """LeRobot 0.6.2 drops video frames when its queue is full. Wait BEFORE add_frame.

    There is exactly one producer; consumers can only free slots during this check.
    This avoids partial rows/camera gaps without replacing encoder implementation.
    """
    encoder = dataset.writer._streaming_encoder
    if encoder is None:
        raise RuntimeError("Streaming encoder is unavailable")
    deadline = time.monotonic() + .5
    while encoder._episode_active:
        if any(not thread.is_alive() for thread in encoder._threads.values()):
            raise RuntimeError("Video encoder stopped unexpectedly")
        if not any(q.full() for q in encoder._frame_queues.values()):
            return
        if time.monotonic() > deadline:
            raise RuntimeError("Video encoder cannot keep up; lower FPS/resolution/camera count")
        time.sleep(.002)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, nargs="?", help="New dataset directory; existing paths are refused")
    parser.add_argument("--task", default="", help="Natural language task description (required for capture)")
    parser.add_argument("--ssh", default="agi@192.168.31.11", help="Robot SSH user@IP; password entered in terminal")
    parser.add_argument("--ssh-control-dir", help="Private directory for a reusable SSH control socket")
    parser.add_argument("--require-ssh-control", action="store_true", help="Do not prompt for credentials when recording")
    parser.add_argument("--known-hosts", help="Optional existing verified SSH known_hosts file")
    parser.add_argument("--remote-env", default="/home/agi/app/env.sh")
    parser.add_argument("--local-gdk", action="store_true", help="Run GDK source locally using an already configured GDK environment")
    parser.add_argument("--gdk-python", default=sys.executable)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration", type=float, default=0, help="Seconds per episode; 0 records until Ctrl+C")
    parser.add_argument("--start-delay", type=int, default=0, help="Countdown seconds after the source is ready")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=CAMERAS[:3])
    parser.add_argument("--max-width", type=int, default=640, help="Preserve aspect ratio; 0 keeps native resolution")
    parser.add_argument("--transport", choices=["jpeg", "raw"], default="jpeg")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--max-sync-ms", type=float, default=50)
    parser.add_argument("--max-action-age-ms", type=float, default=100)
    parser.add_argument("--state-topic", default="/hal/joint_state")
    parser.add_argument("--action-topic", default="/hal/joint_cmd")
    parser.add_argument("--timeout", type=float, default=25)
    parser.add_argument("--probe", action="store_true", help="Read streams and print schema; create no dataset")
    parser.add_argument("--simulate", action="store_true", help="Synthetic test only; never connects to the robot")
    parser.add_argument("--yes", action="store_true", help="Start each episode without waiting for Enter")
    args = parser.parse_args()
    if args.fps < 1 or args.fps > 120 or args.duration < 0 or args.episodes < 1 or args.start_delay < 0 or args.start_delay > 60:
        parser.error("fps must be 1..120, duration >= 0, episodes >= 1")
    if args.max_width < 0 or args.max_width % 2 or not 1 <= args.jpeg_quality <= 100:
        parser.error("max-width must be 0 or an even positive integer; jpeg-quality must be 1..100")
    if args.max_sync_ms <= 0 or args.max_action_age_ms <= 0 or args.timeout <= 0:
        parser.error("Timeout and synchronization tolerances must be positive")
    if len(set(args.cameras)) != len(args.cameras):
        parser.error("Duplicate camera names")
    if not args.probe and (not args.output or not args.task.strip()):
        parser.error("Capture requires output and --task")
    if not args.probe:
        args.output = args.output.expanduser().resolve()
        if args.output.exists():
            parser.error("Output already exists; choose a new directory (no overwrite)")
    import cv2
    cv2.setNumThreads(1)
    if not args.probe:
        import importlib.metadata
        version = importlib.metadata.version("lerobot")
        if version != "0.6.2":
            parser.error("This writer/queue adapter is validated with lerobot==0.6.2; found " + version)
    dataset = None
    manifest = {"simulated": args.simulate, "fps": args.fps, "episodes": [],
                "action_semantics": "Absolute joint position targets (radians) from recorded command topic; causal latest-at-or-before sample, not next state or delta.",
                "state_semantics": "Measured motor_position (radians), ordered by joint_names.",
                "state_topic": args.state_topic, "action_topic": args.action_topic,
                "synchronization": "Uniform source-clock grid; nearest state/image/gripper; causal command. Prefer protobuf header.stamp, fallback to DDS source_ts_ns; selected origins are recorded in schema.timestamp_sources.",
                "max_sync_ms": args.max_sync_ms,
                "gripper_semantics": "Measured left and right gripper driver joint positions in radians, ordered by schema.gripper_names; not gripper commands.",
                "gripper_topics": ["/hal/left_ee_data", "/hal/right_ee_data"],
                "image_processing": {"max_width": args.max_width, "transport": args.transport,
                                     "jpeg_quality": args.jpeg_quality, "video_codec": "libx264", "GPU": False},
                "excluded": "Gripper commands, depth, IMU, lidar, wheel commands, force/torque and proprioception other than positions.",
                "genie_link_button_integration": False}
    failed = False
    try:
        for episode in range(args.episodes if not args.probe else 1):
            source = Source(args)
            count, previous, error = 0, None, None
            try:
                ready, _ = source.receive(args.timeout + 40)
                if ready["kind"] != "ready" or ready["simulated"] != args.simulate:
                    raise ValueError("Unexpected source handshake")
                print(json.dumps(ready, ensure_ascii=False, indent=2), flush=True)
                if args.probe:
                    source.start()
                    for index in range(3):
                        meta, payloads = source.receive(args.timeout)
                        make_frame(meta, payloads, ready, args, index, previous)
                        previous = meta["target_ns"]
                    print("PROBE OK: received synchronized state, causal action and decoded images.")
                    return 0
                if dataset is None:
                    from lerobot.datasets.lerobot_dataset import LeRobotDataset
                    from lerobot.configs.video import RGBEncoderConfig
                    dataset = LeRobotDataset.create(repo_id="local/" + args.output.name, root=args.output,
                        robot_type="G2A" if not args.simulate else "synthetic_test", fps=args.fps,
                        features=feature_schema(ready), use_videos=True, streaming_encoding=True,
                        encoder_queue_maxsize=8, encoder_threads=1, image_writer_threads=0,
                        rgb_encoder=RGBEncoderConfig(vcodec="h264", preset="ultrafast", crf=23))
                    manifest["schema"] = ready
                elif feature_schema(ready) != feature_schema(manifest["schema"]):
                    raise ValueError("Source schema changed between episodes")
                if not args.yes:
                    input("Episode %d ready. Press Enter to start; Ctrl+C while recording stops and saves: " % episode)
                for remaining in range(args.start_delay, 0, -1):
                    print("COUNTDOWN %d" % remaining, flush=True)
                    time.sleep(1)
                source.start()
                target_count = max(1, math.ceil(args.duration * args.fps)) if args.duration else None
                while target_count is None or count < target_count:
                    meta, payloads = source.receive(args.timeout)
                    frame = make_frame(meta, payloads, ready, args, count, previous)
                    ensure_writer_capacity(dataset)
                    dataset.add_frame(frame)
                    previous = meta["target_ns"]
                    count += 1
                    if count == 1:
                        print("RECORDING episode %d%s" % (episode, " [SYNTHETIC TEST]" if args.simulate else ""), flush=True)
                    if count % args.fps == 0:
                        print("episode=%d frames=%d duration=%.1fs" % (episode, count, count / args.fps), flush=True)
            except KeyboardInterrupt:
                print("Stopping capture and saving completed frames...", flush=True)
            except Exception as exc:
                error = str(exc)
                failed = True
                print("ERROR: " + error, file=sys.stderr, flush=True)
            finally:
                source.close()
            if dataset is not None and count:
                # No silently dropped camera frames may be committed as aligned rows.
                encoder = dataset.writer._streaming_encoder
                if any(encoder._dropped_frames.values()):
                    raise RuntimeError("Encoder dropped frames; refusing to save an inconsistent episode")
                dataset.save_episode()
                manifest["episodes"].append({"episode_index": episode, "frames": count,
                    "status": "interrupted_by_error" if error else "saved", "error": error,
                    "last_source_timestamp_ns": previous})
                (args.output / "capture_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
                print("Saved episode %d: %d frames" % (episode, count), flush=True)
            if error or count == 0:
                failed = failed or count == 0
                break
    finally:
        if dataset is not None:
            dataset.finalize()
            (args.output / "capture_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    if dataset is not None and manifest["episodes"]:
        print("Dataset: " + str(args.output), flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        sys.exit(1)
