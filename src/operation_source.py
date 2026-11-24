"""Robot-local manual control with expiring GUI commands; no motion on connect."""
import json,math,os,sys,threading,time,signal
NAMES=[f'idx{i:02d}_body_joint{i}' for i in range(1,6)]+[f'idx{i:02d}_head_joint{i-10}' for i in range(11,14)]+[f'idx{i:02d}_arm_l_joint{i-20}' for i in range(21,28)]+[f'idx{i:02d}_arm_r_joint{i-60}' for i in range(61,68)]

def chassis_vector(x,y,mode,speed):
    if not all(math.isfinite(v) for v in (x,y,speed)) or not 0<speed<=1 or mode not in ('ackermann','crab'):raise ValueError('底盘指令无效')
    r=math.hypot(x,y)
    if r>1.001:raise ValueError('摇杆超出范围')
    if r<.18:return 0.,0.,0.
    gain=(min(r,1)-.18)/(.82*r);x*=gain;y*=gain
    # Narrow axis sectors, matching the reference's straight/spin/strafe wedges.
    if abs(x)<abs(y)*.17:x=0.
    if abs(y)<abs(x)*.17:y=0.
    return (.25*y*speed,0.,-.4*x*speed) if mode=='ackermann' else (.25*y*speed,-.25*x*speed,0.)

class ChassisSession:
    """Track only the task acquired by this process; never cancel unrelated tasks."""
    ACTIVE=(1,2,3,4,5,6)
    def __init__(self,pnc):self.pnc=pnc;self.task_id=None
    def acquire(self):
        previous=self.pnc.get_task_state()
        if previous.state in self.ACTIVE:
            raise RuntimeError(f'底盘已有任务 ID={previous.id}，状态={previous.state}，类型={previous.type}；请结束原任务后重试')
        self.pnc.request_chassis_control(0)
        # The service may acknowledge before its status topic is updated.
        deadline=time.monotonic()+.8
        while True:
            task=self.pnc.get_task_state()
            if task.type==3 and task.state in (1,2):
                self.task_id=task.id;return
            if time.monotonic()>=deadline:
                raise RuntimeError('底盘申请返回成功，但无法确认任务 ID；未发送运动指令，请检查底盘任务状态')
            time.sleep(.02)
    def release(self):
        if self.task_id is None:return
        task=self.pnc.get_task_state()
        if task.id!=self.task_id or task.type!=3 or task.state not in self.ACTIVE:
            self.task_id=None;return
        self.pnc.cancel_task(self.task_id)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            task=self.pnc.get_task_state()
            if task.id!=self.task_id or task.type!=3 or task.state not in self.ACTIVE:
                self.task_id=None;return
            time.sleep(.05)
        raise RuntimeError(f'底盘任务 {self.task_id} 取消后尚未确认结束')

class Inbox:
    def __init__(self):
        self.lock=threading.Lock();self.msg={'kind':'idle'};self.at=time.monotonic();self.closed=False
        threading.Thread(target=self.read,daemon=True).start()
    def read(self):
        try:
            for line in sys.stdin:
                msg=json.loads(line)
                if not isinstance(msg,dict) or msg.get('kind') not in ('idle','stop','quit','arm','base','joint'):raise ValueError('未知控制指令')
                with self.lock:self.msg=msg;self.at=time.monotonic()
                if msg.get('kind')=='quit':break
        finally:
            with self.lock:self.closed=True
    def get(self):
        with self.lock:return dict(self.msg),time.monotonic()-self.at,self.closed

def main():
    def terminate(signum,frame):raise SystemExit(0)
    signal.signal(signal.SIGTERM,terminate)
    signal.signal(signal.SIGHUP,terminate)
    out=os.fdopen(os.dup(1),'w',buffering=1);os.dup2(2,1)
    def emit(kind,**kw):
        try:print(json.dumps({'kind':kind,**kw},ensure_ascii=False),file=out,flush=True)
        except (BrokenPipeError,OSError):pass
    import agibot_gdk as g
    if g.gdk_init()!=g.GDKRes.kSuccess:raise RuntimeError('GDK 初始化失败')
    pnc=None;session=None;base_owned=False;moving=False
    def zero():
        nonlocal moving
        if base_owned:pnc.move_chassis(g.Twist())
        moving=False
    try:
        robot=g.Robot();pnc=g.Pnc();session=ChassisSession(pnc);limits=robot.get_joint_limits();inbox=Inbox()
        armed=False;job=None;last_id=None;tick=0;prev=time.monotonic();last_base=0
        emit('ready')
        while True:
            start=time.monotonic();dt=min(.02,max(.001,start-prev));prev=start
            msg,age,closed=inbox.get();kind=msg.get('kind')
            if closed or kind=='quit':break
            if age>.3:
                zero();armed=False;job=None;emit('error',message='控制通信超时，已停发关节指令并发送底盘零速度');break
            if kind in ('stop','idle'):
                if moving:zero()
                job=None
                if kind=='stop':armed=False
            if tick%10==0:
                status=robot.get_motion_control_status();states=robot.get_joint_states();whole=robot.get_whole_body_status()
                stamp=int(states.get('timestamp',0))
                if not -.5<=(time.time_ns()-stamp)/1e9<=.5:raise RuntimeError('关节反馈过期')
                selected={s['name']:s for s in states['states']}
                positions={n:float(s['motor_position']) for n,s in selected.items() if n in NAMES}
                if any(not math.isfinite(v) for v in positions.values()):raise RuntimeError('关节反馈无效')
                healthy=not whole.get('chassis_error',1) and not whole.get('right_arm_estop',1) and not whole.get('left_arm_estop',1) and status.mode in (0,1) and status.control_mode in (1,3) and status.error_code==0 and not status.collision_pairs_1 and not status.collision_pairs_2 and not any(int(s['error_code']) for s in selected.values())
                reason=f'mode={status.mode}, control_mode={status.control_mode}, error_code={status.error_code}, chassis_error={whole.get("chassis_error")}, left_estop={whole.get("left_arm_estop")}, right_estop={whole.get("right_arm_estop")}, collisions={status.collision_pairs_1}/{status.collision_pairs_2}, joint_errors='+str({n:s.get('error_code') for n,s in selected.items() if s.get('error_code')})
                if armed and not healthy:raise RuntimeError('机器人故障、碰撞或控制模式变化：'+reason)
                emit('state',positions=positions,limits={n:limits[n] for n in positions if n in limits},healthy=bool(healthy),reason=reason)
            # Check the lease again after GDK reads, before any motion publication.
            msg,age,closed=inbox.get();kind=msg.get('kind')
            if closed or age>.3:continue
            if kind=='arm':
                if not healthy:raise RuntimeError('当前机器人状态不允许操作')
                if not armed:armed=True;emit('armed')
            elif kind=='base' and armed:
                job=None
                if not base_owned:
                    try:session.acquire()
                    except Exception as exc:
                        detail=''
                        try:
                            task=pnc.get_task_state()
                            detail=f'；当前底盘任务 ID={task.id}，状态={task.state}，类型={task.type}'
                            if task.type==3 and task.state in (1,2,3,4,5,6):
                                detail+='（已有远程控制任务，请先在原控制端结束遥控后重试）'
                        except Exception:pass
                        raise RuntimeError('申请底盘控制权失败'+detail+'；原始错误：'+str(exc)) from exc
                    base_owned=True
                    # Control acquisition may block. Revalidate the current command.
                    latest,lease_age,ended=inbox.get()
                    if ended or lease_age>.3 or latest!=msg:
                        zero();continue
                if time.monotonic()-last_base>=.05:
                    vx,vy,wz=chassis_vector(float(msg['x']),float(msg['y']),msg['mode'],float(msg['speed']))
                    twist=g.Twist();twist.linear.x=vx;twist.linear.y=vy;twist.angular.z=wz
                    pnc.move_chassis(twist);moving=bool(vx or vy or wz);last_base=time.monotonic()
            elif kind=='joint' and armed:
                if moving:zero()
                jid=int(msg['id']);name=msg['name'];delta=float(msg['delta']);speed=float(msg['speed'])
                if name not in NAMES or name not in positions or name not in limits or not math.isfinite(delta) or abs(delta)>math.radians(10)+1e-8 or not math.isfinite(speed) or not .1<=speed<=1:raise ValueError('关节点动指令无效')
                if jid!=last_id:
                    target=positions[name]+delta;lo=float(limits[name]['min']);hi=float(limits[name]['max'])
                    if not math.isfinite(target) or not lo<=target<=hi:raise ValueError('目标超出关节限位：'+name)
                    job={'name':name,'pos':positions[name],'target':target,'id':jid,'deadline':start+12};last_id=jid
                if job:
                    if start>job['deadline']:raise RuntimeError('关节点动超时')
                    if abs(positions[name]-job['pos'])>.15:raise RuntimeError('关节跟随偏差过大：'+name)
                    step=.15*speed*dt;error=job['target']-job['pos'];job['pos']+=max(-step,min(step,error))
                    req=g.JointServoControlReq();req.control_period=.015;req.joint_names=[name];req.joint_positions=[job['pos']]
                    if robot.joint_servo_control(req)!=0:raise RuntimeError('关节指令发送失败')
                    if abs(error)<=step and abs(positions[name]-job['target'])<.02:
                        emit('done',id=jid);job=None
            tick+=1;time.sleep(max(0,.01-(time.monotonic()-start)))
    except Exception as exc:emit('error',message=str(exc))
    finally:
        try:zero()
        except Exception as exc:emit('error',message='底盘零速度发送失败：'+str(exc))
        try:
            if session is not None:session.release()
        except Exception as exc:emit('error',message='释放底盘任务失败：'+str(exc))
        finally:g.gdk_release()
if __name__=='__main__':main()
