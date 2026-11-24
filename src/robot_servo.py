"""Robot-side replay worker. Emits JSON status and requires GUI heartbeats.

Run via SSH; no command is sent until the input START message after preflight.
"""
import json, math, os, select, sys, threading, time

out=os.fdopen(os.dup(1),'w',buffering=1)
os.dup2(2,1)
def emit(kind, **kwargs):
    print(json.dumps(dict(kind=kind,**kwargs),ensure_ascii=False),file=out,flush=True)

class Inputs:
    def __init__(self):
        self.lock=threading.Lock();self.last=time.monotonic();self.started=False;self.stop=False
        threading.Thread(target=self.read,daemon=True).start()
    def read(self):
        try:
            for line in sys.stdin:
                with self.lock:
                    self.last=time.monotonic()
                    if line.strip()=='START':self.started=True
                    elif line.strip()=='STOP':self.stop=True;return
                    elif line.strip()!='PING':self.stop=True;return
        finally:
            with self.lock:self.stop=True
    def snapshot(self):
        with self.lock:return self.started,self.stop or time.monotonic()-self.last>.6


def checked_positions(robot,names):
    states=robot.get_joint_states(); stamp=int(states.get('timestamp',0)); age=(time.time_ns()-stamp)/1e9
    if stamp<=0 or not -.5<=age<=1.0:raise RuntimeError('机器人关节状态时间戳过期或无效')
    selected={x['name']:x for x in states['states']}
    if any(n not in selected or int(selected[n]['error_code'])!=0 for n in names[:14]):raise RuntimeError('手臂关节状态缺失或存在故障')
    actual={name:float(x['motor_position']) for name,x in selected.items()}
    ee=robot.get_end_state()
    for side in ('left','right'):
        part=ee[side+'_end_state']; ee_names=part['names']; ee_values=part['end_states']
        if not part.get('controlled') or len(ee_names)!=len(ee_values):raise RuntimeError('夹爪状态不可用')
        for name,state in zip(ee_names,ee_values):
            if not state['enable'] or state['err_code']!=0:raise RuntimeError('夹爪未使能或存在故障')
            actual[name]=float(state['position'])
    if any(n not in actual or not math.isfinite(actual[n]) for n in names):raise RuntimeError('关节状态无效')
    return actual

TRACKING_THRESHOLD_RAD = 0.5

def check_tracking(actual, target, names, phase, frame=None, elapsed=None):
    deviations=[abs(actual[n]-target[k]) for k,n in enumerate(names)]
    k=max(range(len(names)),key=lambda k: deviations[k])
    if deviations[k] <= TRACKING_THRESHOLD_RAD:
        return
    detail={'phase':phase,'frame':frame,'elapsed_s':elapsed,'joint':names[k],
            'actual_rad':actual[names[k]],'target_rad':target[k],
            'error_rad':deviations[k],'threshold_rad':TRACKING_THRESHOLD_RAD,
            'violations':[{'joint':n,'actual_rad':actual[n],'target_rad':target[i],
                           'error_rad':deviations[i]} for i,n in enumerate(names) if deviations[i]>TRACKING_THRESHOLD_RAD]}
    emit('tracking_error', **detail)
    location=('，第 %d 帧' % frame) if frame is not None else ''
    raise RuntimeError('%s跟随偏差超限%s：%s，实际 %.6f rad，目标 %.6f rad，偏差 %.6f rad（%.2f°），阈值 %.2f rad' %
                       (phase,location,names[k],actual[names[k]],target[k],deviations[k],math.degrees(deviations[k]),TRACKING_THRESHOLD_RAD))

def check_motion_status(robot):
    status=robot.get_motion_control_status()
    if status.mode not in (0,1) or status.control_mode not in (1,3) or status.error_code != 0 or status.collision_pairs_1 or status.collision_pairs_2:
        raise RuntimeError('机器人运动控制故障、发生碰撞或模式改变')

def main():
    import agibot_gdk as g
    import numpy as np
    config=json.loads(sys.stdin.readline())
    requested_scale=float(config.get('speed_scale',1.0))
    if not math.isfinite(requested_scale) or not .1<=requested_scale<=1.:raise ValueError('回放速度必须在 10%～100% 之间')
    names=config['names'];samples=np.asarray(config['samples'],dtype=float); fps=float(config['fps']);velocity_limits=np.asarray(config.get('velocity_limits',[]),dtype=float)
    if len(names)!=16 or len(set(names))!=16 or samples.ndim!=2 or samples.shape[1]!=16 or len(samples)<2 or not np.isfinite(samples).all() or not 0<fps<=120:
        raise ValueError('轨迹数据无效')
    if velocity_limits.shape!=(16,) or not np.isfinite(velocity_limits).all() or (velocity_limits<=0).any():raise ValueError('关节速度限值无效')
    if g.gdk_init()!=g.GDKRes.kSuccess:raise RuntimeError('GDK 初始化失败')
    try:
        robot=g.Robot(); status=robot.get_motion_control_status()
        if status.mode not in (0,1):raise ValueError('机器人正在执行规划运动，请先停止')
        if status.control_mode not in (1,3) or status.error_code != 0 or status.collision_pairs_1 or status.collision_pairs_2:raise ValueError('机器人控制模式或状态不满足回放条件')
        limits=robot.get_joint_limits()
        # Recorded targets can slightly overshoot the controller boundary.
        # Clamp only sub-0.002 rad deviations and report every adjustment.
        corrections=[];max_allowed_correction=.002
        for j,name in enumerate(names):
            if name not in limits:raise ValueError('设备未提供关节限位：'+name)
            lo=float(limits[name]['min']);hi=float(limits[name]['max'])
            if not math.isfinite(lo) or not math.isfinite(hi) or lo>=hi:raise ValueError('关节限位无效：'+name)
            values=samples[:,j];outside=(values<lo)|(values>hi)
            if outside.any():
                distances=np.maximum(lo-values,values-hi);max_distance=float(distances.max())
                if max_distance>max_allowed_correction:
                    frame=int(np.argmax(distances))
                    raise ValueError('轨迹超出关节限位：%s，第 %d 帧，目标 %.6f，允许范围 [%.6f, %.6f]，超出 %.6f rad' % (name,frame,values[frame],lo,hi,max_distance))
                corrections.append({'joint':name,'frames':int(outside.sum()),'max_rad':max_distance})
                samples[:,j]=np.clip(values,lo,hi)
        actual=checked_positions(robot,names)
        delta=np.abs(np.array([actual[n] for n in names])-samples[0]);
        if delta[:14].max()>.20 or delta[14:].max()>.80:raise ValueError('起始位置偏差过大；请先手动靠近录制起点')
        # Replay at recorded speed only when every joint stays within its URDF limit.
        speeds=np.abs(np.diff(samples,axis=0))*fps
        peak=float(speeds.max()) if speeds.size else 0.0
        per_joint=np.max(speeds,axis=0)
        ratios=velocity_limits/np.maximum(per_joint,1e-12)
        scale=min(requested_scale,float(np.min(ratios)))
        if scale<.01:raise ValueError('轨迹速度过高，不能安全回放')
        speed_limited=[{'joint':names[j],'recorded':float(per_joint[j]),'limit':float(velocity_limits[j])}
                       for j in range(len(names)) if per_joint[j]>velocity_limits[j]]
        inp=Inputs();duration=(len(samples)-1)/(fps*scale)
        emit('ready',duration=duration,scale=scale,max_start_delta=float(delta.max()),max_recorded_speed=peak,limit_corrections=corrections,speed_limited=speed_limited)
        while True:
            started,stop=inp.snapshot()
            if stop:emit('stopped',reason='心跳中断');return
            if started:break
            time.sleep(.01)
        # Re-check pose immediately after user confirmation to prevent stale preflight.
        actual=checked_positions(robot,names)
        if any(n not in actual or not math.isfinite(actual[n]) or abs(actual[n]-samples[0,i])>(.20 if i<14 else .80) for i,n in enumerate(names)):raise ValueError('起始姿态偏差过大，请手动靠近起点')
        status=robot.get_motion_control_status()
        if status.mode not in (0,1) or status.control_mode not in (1,3) or status.error_code != 0 or status.collision_pairs_1 or status.collision_pairs_2:raise ValueError('机器人控制状态已变化，停止回放')
        initial=np.array([actual[n] for n in names]); delta=np.abs(initial-samples[0]); transition=max(1.0,float(delta.max())/.10)
        emit('transition',duration=transition)
        begin=time.monotonic();tick=0
        while True:
            _,stop=inp.snapshot()
            if stop:emit('stopped',reason='停止或心跳中断');return
            delay=begin+tick*.01-time.monotonic()
            if delay>0:time.sleep(delay)
            elapsed=time.monotonic()-begin
            if elapsed-tick*.01>.05:raise TimeoutError('起始位姿过渡延迟超限')
            _,stop=inp.snapshot()
            if stop:emit('stopped',reason='停止或心跳中断');return
            t=min(1.,elapsed/transition); pos=(initial+t*(samples[0]-initial)).tolist()
            req=g.JointServoControlReq();req.control_period=.015;req.joint_names=names;req.joint_positions=pos
            if tick%25==0:
                check_motion_status(robot)
                current=checked_positions(robot,names)
                check_tracking(current,pos,names,'起始位姿过渡',elapsed=elapsed)
                _,stop=inp.snapshot()
                if stop:emit('stopped',reason='停止或心跳中断');return
            if robot.joint_servo_control(req)!=0:raise RuntimeError('起始位姿伺服发送失败')
            if t>=1.:break
            tick+=1
        actual=checked_positions(robot,names)
        if any(n not in actual or not math.isfinite(actual[n]) or abs(actual[n]-samples[0,i])>.05 for i,n in enumerate(names)):raise ValueError('未到达起始姿态，停止回放')
        emit('started')
        begin=time.monotonic();tick=0
        while True:
            _,stop=inp.snapshot()
            if stop:emit('stopped',reason='停止或心跳中断');break
            deadline=begin+tick*.01; delay=deadline-time.monotonic()
            if delay>0:time.sleep(delay)
            elapsed=time.monotonic()-begin
            if elapsed-tick*.01>.05:raise TimeoutError('控制循环延迟超限，停止发送')
            _,stop=inp.snapshot()
            if stop:emit('stopped',reason='停止或心跳中断');break
            u=min(len(samples)-1,elapsed*fps*scale);i=int(u);j=min(i+1,len(samples)-1)
            pos=(samples[i]+(u-i)*(samples[j]-samples[i])).tolist()
            req=g.JointServoControlReq();req.control_period=.015;req.joint_names=names;req.joint_positions=pos
            if tick%25==0:
                check_motion_status(robot)
                actual=checked_positions(robot,names)
                check_tracking(actual,pos,names,'动作回放',frame=int(u),elapsed=elapsed)
                _,stop=inp.snapshot()
                if stop:emit('stopped',reason='停止或心跳中断');break
            result=robot.joint_servo_control(req)
            if result!=0:raise RuntimeError('伺服指令失败：'+str(result))
            if tick%10==0:emit('progress',frame=int(u))
            if u>=len(samples)-1:emit('finished');break
            tick+=1
    finally:
        g.gdk_release()

if __name__=='__main__':
    try:main()
    except Exception as exc:emit('error',message=str(exc));sys.exit(1)
