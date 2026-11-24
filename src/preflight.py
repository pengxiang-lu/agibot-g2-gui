"""Inspect recorded arm and gripper positions without controlling hardware."""
import json
from pathlib import Path
import numpy as np
import pyarrow.parquet as pq

ARM_NAMES=[f'idx{i:02d}_arm_l_joint{i-20}' for i in range(21,28)]+[f'idx{i:02d}_arm_r_joint{i-60}' for i in range(61,68)]
GRIPPER_NAMES=['idx31_gripper_l_inner_joint1','idx71_gripper_r_inner_joint1']

def inspect_dataset(root):
    root=Path(root)
    manifest=json.loads((root/'capture_manifest.json').read_text())
    info=json.loads((root/'meta/info.json').read_text())
    schema=manifest.get('schema',{})
    joint_names=schema.get('joint_names',[]);gripper_names=schema.get('gripper_names',[])
    missing=[name for name in ARM_NAMES if name not in joint_names]
    if missing:raise ValueError('记录缺少手臂关节：'+', '.join(missing))
    if gripper_names!=GRIPPER_NAMES:raise ValueError('记录缺少左右夹爪实测位置或夹爪名称不匹配')
    fps=float(info['fps'])
    if not 0<fps<=120:raise ValueError('采集帧率无效')
    paths=sorted((root/'data').rglob('*.parquet'))
    if not paths:raise ValueError('未找到动作数据')
    rows=[]
    for p in paths:rows.extend(pq.read_table(p,columns=['action','observation.gripper','frame_index','episode_index']).to_pylist())
    rows.sort(key=lambda x:(x['episode_index'],x['frame_index']))
    if not rows or {r['episode_index'] for r in rows}!={0} or [r['frame_index'] for r in rows]!=list(range(len(rows))):raise ValueError('帧序号不连续，或包含多个 episode')
    actions=np.asarray([r['action'] for r in rows],dtype=np.float64)
    grippers=np.asarray([r['observation.gripper'] for r in rows],dtype=np.float64)
    if actions.shape!=(len(rows),len(joint_names)) or grippers.shape!=(len(rows),2) or not np.isfinite(actions).all() or not np.isfinite(grippers).all():raise ValueError('动作或夹爪位置无效')
    arm=actions[:,[joint_names.index(name) for name in ARM_NAMES]]
    max_speed=float(np.max(np.abs(np.diff(arm,axis=0)))*fps) if len(rows)>1 else 0.0
    return {'frames':len(rows),'fps':fps,'max_arm_recorded_speed':max_speed,'gripper_min':float(grippers.min()),'gripper_max':float(grippers.max()),'simulated':bool(manifest.get('simulated'))}
