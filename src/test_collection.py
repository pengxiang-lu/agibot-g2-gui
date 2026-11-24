"""Tests for causal alignment, joint order, framing and interrupted recordings."""
import argparse
import io
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

import collect
import gdk_source


class AlignmentTests(unittest.TestCase):
    def test_hal_header_timestamp_when_dds_timestamp_is_zero(self):
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1795424000, nanosec=123)))
        info = SimpleNamespace(source_ts_ns=0)
        self.assertEqual(gdk_source.message_timestamp(message, info),
                         (1795424000000000123, "protobuf.header.stamp"))
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        info.source_ts_ns = 456
        self.assertEqual(gdk_source.message_timestamp(message, info), (456, "dds.source_ts_ns"))

    def test_causal_command_ignores_future_and_arrival_order(self):
        samples = [(103, ["a"], [3]), (98, ["a"], [2]), (90, ["a"], [1])]
        self.assertEqual(gdk_source.select_sample(samples, 100, causal=True, limit_ns=10)[0], 98)
        with self.assertRaises(RuntimeError):
            gdk_source.select_sample(samples, 200, causal=True, limit_ns=10)
        with self.assertRaises(RuntimeError):
            gdk_source.select_sample(samples, 80, causal=True, limit_ns=10)

    def test_joint_order_and_missing_joint(self):
        self.assertEqual(gdk_source.ordered_values(["b", "a"], [2, 1], ["a", "b"]), [1, 2])
        with self.assertRaises(RuntimeError):
            gdk_source.ordered_values(["a"], [1], ["a", "b"])

    def test_binary_protocol_handles_partial_writes(self):
        class ShortWrites(io.BytesIO):
            def write(self, data):
                return super().write(data[:3])
        stream = ShortWrites()
        gdk_source.send(stream, {"kind": "frame"}, [b"abcde", b"123456789"])
        stream.seek(0)
        meta, images = collect.read_packet(stream)
        self.assertEqual(meta["kind"], "frame")
        self.assertEqual(images, [b"abcde", b"123456789"])

    def test_future_action_is_rejected(self):
        args = argparse.Namespace(cameras=["head_color"], simulate=True, transport="raw", task="test",
                                  fps=10, max_sync_ms=50, max_action_age_ms=100)
        synthetic = gdk_source.Synthetic(args)
        ready = {"cameras": args.cameras, "shapes": synthetic.shapes, "transport": "raw",
                 "joint_names": synthetic.names}
        meta, images = synthetic.frame(1_000_000_000, 0)
        meta["action_ns"] = meta["target_ns"] + 1
        with self.assertRaises(ValueError):
            collect.make_frame(meta, images, ready, args, 0, None)

    def test_interrupted_stream_saves_only_valid_frames_and_returns_failure(self):
        args = argparse.Namespace(cameras=["head_color"], simulate=True)
        synthetic = gdk_source.Synthetic(args)

        class BrokenSource:
            def __init__(self, *unused):
                self.index = -1

            def receive(self, timeout):
                if self.index == -1:
                    self.index = 0
                    return {"kind": "ready", "simulated": True, "cameras": args.cameras,
                            "shapes": synthetic.shapes, "transport": "raw", "joint_names": synthetic.names,
                            "omitted_state_joints": []}, []
                if self.index == 3:
                    raise RuntimeError("injected network interruption")
                value = synthetic.frame(1_000_000_000 + self.index * 100_000_000, self.index)
                self.index += 1
                return value

            def start(self):
                pass

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "interrupted"
            argv = ["collect.py", str(output), "--simulate", "--task", "test", "--fps", "10", "--yes"]
            with mock.patch.object(collect, "Source", BrokenSource), mock.patch("sys.argv", argv):
                self.assertEqual(collect.main(), 1)
            manifest = json.loads((output / "capture_manifest.json").read_text())
            self.assertEqual(manifest["episodes"][0]["frames"], 3)
            self.assertEqual(manifest["episodes"][0]["status"], "interrupted_by_error")
            from verify_dataset import verify
            verify(output)


if __name__ == "__main__":
    unittest.main()
