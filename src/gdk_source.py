"""Read-only GDK source. Launched by collect.py, locally or through SSH.

stdout is a length-framed protocol; SDK output is redirected to stderr before import.
No publishers, motion calls, mode changes or camera configuration writes are used.
"""
import argparse
import collections
import concurrent.futures
import json
import os
import queue
import select
import struct
import sys
import threading
import time
from types import SimpleNamespace

CAMERAS = {
    "head_color": "kHeadColor", "hand_left_color": "kHandLeftColor",
    "hand_right_color": "kHandRightColor", "head_stereo_left": "kHeadStereoLeft",
    "head_stereo_right": "kHeadStereoRight", "head_left_fisheye": "kHeadLeftFisheye",
    "head_right_fisheye": "kHeadRightFisheye", "head_back_fisheye": "kHeadBackFisheye",
}
GRIPPER_NAMES = ['idx31_gripper_l_inner_joint1', 'idx71_gripper_r_inner_joint1']


def send(out, metadata, images=()):
    metadata["payload_sizes"] = [len(x) for x in images]
    header = json.dumps(metadata, separators=(",", ":"), allow_nan=False).encode()
    for data in [struct.pack("!I", len(header)), header, *images]:
        remaining = memoryview(data)
        while remaining:
            written = out.write(remaining)
            if not written:
                raise BrokenPipeError("Output closed")
            remaining = remaining[written:]
    out.flush()


def select_sample(samples, target, causal=False, limit_ns=100_000_000):
    eligible = [x for x in samples if not causal or x[0] <= target]
    if not eligible:
        raise RuntimeError("No causal command at or before the sample time")
    result = max(eligible, key=lambda x: x[0]) if causal else min(eligible, key=lambda x: abs(x[0] - target))
    if abs(result[0] - target) > limit_ns:
        raise RuntimeError("Stale/misaligned telemetry: delta=%.1f ms" % ((result[0] - target) / 1e6))
    return result


def ordered_values(names, values, expected):
    if len(names) != len(values) or len(names) != len(set(names)):
        raise RuntimeError("Invalid joint names/values")
    mapping = dict(zip(names, values))
    missing = set(expected) - mapping.keys()
    if missing:
        raise RuntimeError("Missing joints: " + str(sorted(missing)))
    return [float(mapping[x]) for x in expected]


def nearest_camera_frame(camera, camera_type, name, target, max_sync_ms, wait_ms=200):
    """Briefly wait for late frames, but never relax the alignment tolerance."""
    deadline = time.monotonic() + wait_ms / 1000.0
    limit_ns = max_sync_ms * 1_000_000
    attempts = 0
    while True:
        remaining_ms = max(1.0, (deadline - time.monotonic()) * 1000)
        im = camera.get_nearest_image(camera_type, target, min(wait_ms, remaining_ms))
        stamp = int(im.timestamp_ns)
        delta = stamp - target
        attempts += 1
        if abs(delta) <= limit_ns:
            if attempts > 1:
                print("CAMERA_WAIT %s attempts=%d alignment_ms=%.1f" % (name, attempts, delta / 1e6),
                      file=sys.stderr, flush=True)
            return im
        if delta > limit_ns or time.monotonic() >= deadline:
            raise RuntimeError("Camera %s is stale/misaligned: %.1f ms (waited up to %d ms)" %
                               (name, delta / 1e6, wait_ms))
        time.sleep(min(.01, max(0, deadline - time.monotonic())))


def message_timestamp(message, info):
    """HAL publishers can leave DDS source_ts_ns at zero; their Header carries time."""
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec, nsec = int(getattr(stamp, "sec", 0)), int(getattr(stamp, "nanosec", 0))
    if sec > 0 and 0 <= nsec < 1_000_000_000:
        return sec * 1_000_000_000 + nsec, "protobuf.header.stamp"
    return int(info.source_ts_ns), "dds.source_ts_ns"


def rgb_image(image):
    import cv2
    import numpy as np
    data = np.asarray(image.data, dtype=np.uint8)
    encoding = str(image.encoding).split(".")[-1]
    fmt = str(image.color_format).split(".")[-1]
    h, w = int(image.height), int(image.width)
    if encoding in ("JPEG", "PNG"):
        decoded = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if decoded is None:
            raise RuntimeError("Invalid compressed camera frame")
        return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
    if encoding != "UNCOMPRESSED" or int(image.bit_depth) != 8:
        raise RuntimeError("Unsupported camera encoding/depth: %s/%s" % (encoding, image.bit_depth))
    if fmt in ("RGB", "BGR", "RGBA", "BGRA"):
        channels = 4 if "A" in fmt else 3
        data = data.reshape(h, w, channels)
        return data if fmt == "RGB" else cv2.cvtColor(data, getattr(cv2, "COLOR_" + fmt + "2RGB"))
    if fmt == "GRAY8":
        return cv2.cvtColor(data.reshape(h, w), cv2.COLOR_GRAY2RGB)
    codes = {"NV12": cv2.COLOR_YUV2RGB_NV12, "NV21": cv2.COLOR_YUV2RGB_NV21,
             "YUV420": cv2.COLOR_YUV2RGB_I420}
    if fmt in codes:
        return cv2.cvtColor(data.reshape(h * 3 // 2, w), codes[fmt])
    raise RuntimeError("Unsupported color format: " + fmt)


class Live:
    def __init__(self, args):
        import agibot_gdk as g
        from genie_msgs_pb.msg.JointState_pb2 import JointState
        from genie_msgs_pb.msg.JointCommand_pb2 import JointCommand
        from genie_msgs_pb.msg.EndState_pb2 import EndState
        self.g, self.args = g, args
        self.lock = threading.Lock()
        self.buffers = {key: collections.deque(maxlen=6000) for key in ('state','action','gripper_left','gripper_right')}
        self.gripper_names = GRIPPER_NAMES[:]
        self.callback_error = None
        self.timestamp_sources = {}
        if g.gdk_init() != g.GDKRes.kSuccess:
            raise RuntimeError("GDK initialization failed")
        self.node = g.dds.Node("lerobot_readonly_%d" % os.getpid())
        qos = g.dds.GDKQoS(depth=256, reliability=g.dds.GDKQoSReliability.BEST_EFFORT,
                            durability=g.dds.GDKQoSDurability.VOLATILE)
        self.subs = [self.node.create_subscriber(args.state_topic, JointState, qos, self.callback("state")),
                     self.node.create_subscriber(args.action_topic, JointCommand, qos, self.callback("action")),
                     self.node.create_subscriber('/hal/left_ee_data', EndState, qos, self.callback('gripper_left')),
                     self.node.create_subscriber('/hal/right_ee_data', EndState, qos, self.callback('gripper_right'))]
        self.types = {name: getattr(g.CameraType, CAMERAS[name]) for name in args.cameras}
        self.camera = g.Camera(list(self.types.values()))
        self.encoders = concurrent.futures.ThreadPoolExecutor(max_workers=min(3,len(self.types)))
        deadline = time.monotonic() + args.timeout
        last_error = "Waiting for telemetry"
        while time.monotonic() < deadline:
            try:
                with self.lock:
                    if self.callback_error:
                        raise RuntimeError(self.callback_error)
                    if not all(self.buffers.values()):
                        raise RuntimeError("Missing telemetry: " + ', '.join(k for k, v in self.buffers.items() if not v) + "; check robot and teleoperation state")
                    state = self.buffers["state"][-1]
                    action = self.buffers["action"][-1]
                    left = self.buffers['gripper_left'][-1]
                    right = self.buffers['gripper_right'][-1]
                self.names = [x for x in state[1] if x in set(action[1])]
                if not self.names:
                    raise RuntimeError("State and command have no common joint names")
                # Use only actual shared joints, disclose all omitted joints in ready metadata.
                self.omitted = sorted(set(state[1]) - set(self.names))
                images = [self.camera.get_latest_image(t, 300.0) for t in self.types.values()]
                ordered_values(left[1], left[2], [self.gripper_names[0]])
                ordered_values(right[1], right[2], [self.gripper_names[1]])
                stamps = [state[0], action[0], left[0], right[0]] + [int(im.timestamp_ns) for im in images]
                if min(stamps) <= 0 or max(stamps) - min(stamps) > 500_000_000:
                    raise RuntimeError("Streams are stale or timestamps do not share a clock: " + str(stamps))
                self.shapes = [list(self.encode(im)[1]) for im in images]
                return
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(.1)
        raise RuntimeError("Read-only stream probe timed out: " + last_error)

    def callback(self, kind):
        def on_message(message, info):
            try:
                stamp, clock_source = message_timestamp(message, info)
                if kind.startswith('gripper_'):
                    values = [x.position for x in message.end_state]
                else:
                    values = list(message.motor_position if kind == "state" else message.position)
                names = list(message.name)
                if not stamp or not names or len(names) != len(values):
                    raise RuntimeError("Invalid %s: stamp=%s names=%d positions=%d fields=%s header=%s" %
                                       (kind, stamp, len(names), len(values),
                                        [(f.name, len(v) if hasattr(v, '__len__') else str(v)) for f, v in message.ListFields()],
                                        str(message.header)))
                with self.lock:
                    old_source = self.timestamp_sources.setdefault(kind, clock_source)
                    if old_source != clock_source:
                        raise RuntimeError("Timestamp source changed for " + kind)
                    self.buffers[kind].append((stamp, names, values))
            except Exception as exc:
                with self.lock:
                    self.callback_error = str(exc)
        return on_message

    def encode(self, image):
        import cv2
        rgb = rgb_image(image)
        if self.args.max_width and rgb.shape[1] > self.args.max_width:
            w = self.args.max_width
            h = max(2, round(rgb.shape[0] * w / rgb.shape[1]) // 2 * 2)
            rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
        if rgb.shape[0] % 2 or rgb.shape[1] % 2:
            raise RuntimeError("Video dimensions must be even")
        if self.args.transport == "raw":
            return rgb.tobytes(), rgb.shape
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return encoded.tobytes(), rgb.shape

    def anchor(self):
        cam_type = next(iter(self.types.values()))
        old = int(self.camera.get_latest_image(cam_type, 1000.0).timestamp_ns)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            new = int(self.camera.get_latest_image(cam_type, 100.0).timestamp_ns)
            if new > old:
                return new
            time.sleep(.005)
        raise RuntimeError("Anchor camera is frozen")

    def sample(self, target, index):
        with self.lock:
            if self.callback_error:
                raise RuntimeError(self.callback_error)
            state = select_sample(list(self.buffers["state"]), target, limit_ns=int(self.args.max_sync_ms * 1e6))
            action = select_sample(list(self.buffers["action"]), target, causal=True,
                                   limit_ns=int(self.args.max_action_age_ms * 1e6))
            left = select_sample(list(self.buffers['gripper_left']), target, limit_ns=int(self.args.max_sync_ms * 1e6))
            right = select_sample(list(self.buffers['gripper_right']), target, limit_ns=int(self.args.max_sync_ms * 1e6))
        # Fetch every camera before encoding; serial encoding lets early cameras age
        # while later camera frames can be evicted from the GDK ring buffer.
        frames = [nearest_camera_frame(self.camera, cam_type, name, target, self.args.max_sync_ms)
                  for name, cam_type in self.types.items()]
        stamps = [int(im.timestamp_ns) for im in frames]
        # Copy before returning; the SDK can reuse its camera buffers while the
        # independent encoding and network-sending thread is still working.
        frames = [SimpleNamespace(data=im.data.copy(), encoding=im.encoding,
                  color_format=im.color_format, height=im.height, width=im.width,
                  bit_depth=im.bit_depth) for im in frames]
        meta = {"kind": "frame", "index": index, "target_ns": target,
                "state_ns": state[0], "action_ns": action[0], "camera_ns": stamps,
                "state": ordered_values(state[1], state[2], self.names),
                "action": ordered_values(action[1], action[2], self.names),
                "gripper": ordered_values(left[1],left[2],[self.gripper_names[0]]) + ordered_values(right[1],right[2],[self.gripper_names[1]]),
                "gripper_ns": [left[0],right[0]]}
        return meta, frames

    def encode_sample(self, frames):
        results = list(self.encoders.map(self.encode, frames))
        images = []
        for (payload, new_shape), shape in zip(results, self.shapes):
            if list(new_shape) != shape:
                raise RuntimeError("Camera shape changed")
            images.append(payload)
        return images

    def frame(self, target, index):
        metadata, frames = self.sample(target, index)
        return metadata, self.encode_sample(frames)

    def close(self):
        # Native SDK teardown can hang on unavailable streams; parent has a bounded shutdown timeout.
        if hasattr(self, "encoders"):
            self.encoders.shutdown(wait=True)
        if hasattr(self, "camera"):
            self.camera.close_camera()
        self.g.gdk_release()


class Synthetic:
    def __init__(self, args):
        self.args = args
        self.names = ["test_joint_%d" % i for i in range(3)]
        self.gripper_names = GRIPPER_NAMES[:]
        self.omitted = []
        self.timestamp_sources = {"state": "synthetic", "action": "synthetic"}
        self.shapes = [[48, 64, 3] for _ in args.cameras]

    def anchor(self):
        return time.time_ns()

    def frame(self, target, index):
        import numpy as np
        value = index % 200
        images = [np.full(shape, value + i, dtype=np.uint8).tobytes() for i, shape in enumerate(self.shapes)]
        return {"kind": "frame", "index": index, "target_ns": target, "state_ns": target - 1_000_000,
                "action_ns": target - 2_000_000, "camera_ns": [target] * len(images),
                "state": [index * .001, .1, .2], "action": [index * .001 + .01, .11, .21],
                "gripper": [-.6 + .2 * np.sin(index * .1), -.4 + .2 * np.cos(index * .1)],
                "gripper_ns": [target - 1_000_000, target - 2_000_000]}, images

    def sample(self, target, index):
        return self.frame(target, index)

    def encode_sample(self, frames):
        return frames

    def close(self):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=["head_color", "hand_left_color", "hand_right_color"])
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=25)
    parser.add_argument("--max-sync-ms", type=float, default=50)
    parser.add_argument("--max-action-age-ms", type=float, default=100)
    parser.add_argument("--max-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--transport", choices=["raw", "jpeg"], default="jpeg")
    parser.add_argument("--state-topic", default="/hal/joint_state")
    parser.add_argument("--action-topic", default="/hal/joint_cmd")
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()
    import cv2
    cv2.setNumThreads(1)
    out = os.fdopen(os.dup(1), "wb", buffering=0)
    os.dup2(2, 1)
    source = None
    try:
        source = Synthetic(args) if args.simulate else Live(args)
        send(out, {"kind": "ready", "joint_names": source.names, "gripper_names": source.gripper_names, "omitted_state_joints": source.omitted,
                   "cameras": args.cameras, "shapes": source.shapes,
                   "timestamp_sources": source.timestamp_sources,
                   "transport": "raw" if args.simulate else args.transport, "simulated": args.simulate})
        if sys.stdin.readline().strip() != "START":
            return
        anchor, start = source.anchor(), time.monotonic()
        pending = queue.Queue(maxsize=8)
        stopped = threading.Event()
        def sampler():
            index = 0
            try:
                while not stopped.is_set():
                    deadline = start + index / args.fps + .03
                    if stopped.wait(max(0, deadline - time.monotonic())):
                        break
                    if time.monotonic() - deadline > .5:
                        raise RuntimeError("Capture cannot keep up (>500 ms lag); lower FPS/camera resolution/count")
                    target = anchor + round(index * 1_000_000_000 / args.fps)
                    sample = source.sample(target, index)
                    try:
                        pending.put(("frame", sample), timeout=.05)
                    except queue.Full:
                        raise RuntimeError("Encoder/network cannot keep up; lower FPS/camera resolution/count")
                    index += 1
            except Exception as exc:
                # Do not silently discard a sample or change the fixed-rate timeline.
                while not stopped.is_set():
                    try:
                        pending.put(("error", str(exc)), timeout=.1)
                        break
                    except queue.Full:
                        continue
        worker = threading.Thread(target=sampler, daemon=True)
        worker.start()
        sent = 0
        try:
            while True:
                if select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.readline().strip() != "CONTINUE":
                        break
                try:
                    kind, sample = pending.get(timeout=.05)
                except queue.Empty:
                    continue
                if kind == "error":
                    raise RuntimeError(sample)
                metadata, frames = sample
                send(out, metadata, source.encode_sample(frames))
                sent += 1
        finally:
            stopped.set()
            worker.join(timeout=3)
            if worker.is_alive():
                raise RuntimeError("Camera sampling thread failed to stop")
        send(out, {"kind": "end", "frames": sent})
    except BrokenPipeError:
        pass
    except Exception as exc:
        send(out, {"kind": "error", "message": str(exc)})
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    main()
