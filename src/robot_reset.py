"""Independent GDK planned reset; --probe validates without publishing motion."""
import json, math, sys, time
from pathlib import Path

CONFIG = '/home/agi/app/share/teleop_it/yaml/G2_t2v2_crs.yaml'
AID = 'G2A0005C600872'
TOLERANCE = .03

def load_target(whole):
    import yaml
    identity=yaml.safe_load(Path('/data/version/aid_info.yaml').read_text())
    if identity.get('AID')!=AID:
        raise RuntimeError('此复位配置仅为已核实的 G2A0005C600872 配置；其他机器人需先核实型号与姿态')
    config=yaml.safe_load(Path(CONFIG).read_text())
    if whole.get('left_end_model')!='omnipicker' or whole.get('right_end_model')!='omnipicker':
        raise RuntimeError('夹爪型号已变化，请重新核实复位姿态')
    poses=[p for p in config['reset']['reset_pose_list'] if p['pose_id']==11]
    if len(poses)!=1:raise RuntimeError('未找到唯一的 OmniPicker 官方复位姿态 pose 11')
    names=[];targets=[]
    for key,field,count in [('left','left_arm_joint_names',7),('right','right_arm_joint_names',7),('body','body_joint_names',5),('head','head_joint_names',3)]:
        group=config['joint_names'][field];values=poses[0][key]
        if len(group)!=count or len(values)!=count:raise RuntimeError('复位配置关节数量不匹配：'+key)
        names.extend(group);targets.extend(float(x) for x in values)
    if len(set(names))!=22 or not all(math.isfinite(v) for v in targets):raise RuntimeError('复位配置无效')
    return names,targets

def positions(robot,names):
    states=robot.get_joint_states();stamp=int(states.get('timestamp',0))
    if stamp<=0 or not -.5<=(time.time_ns()-stamp)/1e9<=.5:raise RuntimeError('关节反馈过期')
    joints={x['name']:x for x in states['states']};values=[]
    for n in names:
        x=joints.get(n)
        if not x or int(x['error_code']):raise RuntimeError('关节反馈缺失或故障：'+n)
        value=float(x['motor_position'])
        if not math.isfinite(value):raise RuntimeError('关节反馈无效：'+n)
        values.append(value)
    return values

def check_status(robot):
    s=robot.get_motion_control_status()
    if s.mode not in (0,1) or s.control_mode not in (1,3) or s.error_code or s.collision_pairs_1 or s.collision_pairs_2:
        raise RuntimeError('机器人正执行其他规划、存在故障或控制模式不允许复位')
    whole=robot.get_whole_body_status()
    for key in ('left_arm_error','right_arm_error','left_arm_estop','right_arm_estop','waist_error','lift_error','neck_error','chassis_error'):
        if whole.get(key,True):raise RuntimeError('机器人状态不允许复位：'+key)
    return whole

def main(probe=False):
    import agibot_gdk as g
    if g.gdk_init()!=g.GDKRes.kSuccess:raise RuntimeError('GDK 初始化失败')
    try:
        robot=g.Robot();whole=check_status(robot);names,target=load_target(whole)
        limits=robot.get_joint_limits()
        for n,v in zip(names,target):
            limit=limits.get(n,{})
            lo=float(limit.get('min',float('nan')));hi=float(limit.get('max',float('nan')))
            if not math.isfinite(lo) or not math.isfinite(hi) or not lo<=v<=hi:raise RuntimeError('复位目标超出限位或限位缺失：'+n)
        current=positions(robot,names);delta=max(abs(a-b) for a,b in zip(current,target))
        lifetime=max(5.,delta/.2+10.)
        if lifetime>50:raise RuntimeError('当前姿态离复位目标过远，请先检查姿态')
        if probe:return f'独立 GDK 复位检查通过：OmniPicker pose 11，22 个关节，最大起点偏差 {delta:.4f} rad；未发送动作'
        if delta<=TOLERANCE:return '实测关节已在复位目标范围内，无需移动'
        # A blocking planned request, not a teleoperation event or a raw servo stream.
        check_status(robot)
        req=g.JointControlReq();req.joint_names=names;req.joint_positions=target
        req.joint_velocities=[.2]*len(names);req.life_time=lifetime
        req.detail='G2 workstation independent reset, OmniPicker pose 11'
        result=robot.joint_control_request(req)
        if result!=0:raise RuntimeError('GDK 复位规划执行失败：'+str(result))
        deadline=time.monotonic()+2;error=float('inf');worst=0
        while time.monotonic()<deadline:
            check_status(robot);actual=positions(robot,names)
            errors=[abs(a-b) for a,b in zip(actual,target)];worst=max(range(len(names)),key=lambda i:errors[i]);error=errors[worst]
            if error<=TOLERANCE:return f'复位完成，已核对实测关节位置（最大偏差 {error:.4f} rad）'
            time.sleep(.1)
        raise RuntimeError(f'规划接口返回成功，但未确认到达目标：{names[worst]} 偏差 {error:.4f} rad；不会自动重试')
    finally:g.gdk_release()

if __name__=='__main__':
    try:print('RESET_RESULT '+json.dumps({'ok':True,'message':main('--probe' in sys.argv)},ensure_ascii=False),flush=True)
    except Exception as exc:
        print('RESET_RESULT '+json.dumps({'ok':False,'message':str(exc)},ensure_ascii=False),flush=True);sys.exit(1)
