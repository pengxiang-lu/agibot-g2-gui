#!/usr/bin/env python3
"""Load a local dataset with the official LeRobot reader and decode episode boundaries."""
import argparse
import json
from pathlib import Path
import tempfile


def verify(root):
    import datasets.config
    import numpy as np
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = Path(root).resolve()
    manifest = json.loads((root / "capture_manifest.json").read_text())
    previous_cache = datasets.config.HF_DATASETS_CACHE
    with tempfile.TemporaryDirectory(prefix="lerobot_verify_") as cache:
        datasets.config.HF_DATASETS_CACHE = Path(cache)
        try:
            ds = LeRobotDataset(repo_id="local/" + root.name, root=root, video_backend="pyav")
            expected = sum(x["frames"] for x in manifest["episodes"])
            assert len(ds) == expected and ds.num_episodes == len(manifest["episodes"])
            offset = 0
            for episode in manifest["episodes"]:
                for index in sorted({offset, offset + episode["frames"] - 1}):
                    frame = ds[index]
                    assert len(frame["action"]) == len(manifest["schema"]["joint_names"])
                    for cam in manifest["schema"]["cameras"]:
                        image = frame["observation.images." + cam]
                        assert image.shape[0] == 3 and bool(image.isfinite().all())
                offset += episode["frames"]
        finally:
            datasets.config.HF_DATASETS_CACHE = previous_cache
    # Inspect every telemetry timestamp, independent of the video reader's tolerance.
    rows = []
    for path in sorted((root / "data").rglob("*.parquet")):
        rows.extend(pq.read_table(path).to_pylist())
    for episode in manifest["episodes"]:
        selected = [r for r in rows if r["episode_index"] == episode["episode_index"]]
        assert len(selected) == episode["frames"]
        scalar = lambda value: int(np.asarray(value).reshape(-1)[0])
        stamps = np.array([scalar(r["source_timestamp_ns"]) for r in selected], dtype=np.int64)
        if len(stamps) > 1:
            assert np.max(np.abs(np.diff(stamps) - 1e9 / manifest["fps"])) <= 2
        assert all(scalar(r["action_timestamp_ns"]) <= scalar(r["source_timestamp_ns"]) for r in selected)
        if manifest["schema"].get("gripper_names"):
            for row in selected:
                assert len(row["observation.gripper"]) == 2 and np.isfinite(row["observation.gripper"]).all()
                for side in ("left", "right"):
                    assert abs(scalar(row["gripper_" + side + "_timestamp_ns"]) - scalar(row["source_timestamp_ns"])) <= manifest["max_sync_ms"] * 1e6
    print(json.dumps({"verified": True, "format": "LeRobot v3.0", "episodes": len(manifest["episodes"]),
                      "frames": len(rows), "simulated": manifest["simulated"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    verify(parser.parse_args().dataset)
