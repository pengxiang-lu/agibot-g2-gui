"""Run on G2 head board using the currently deployed native ROS services."""
import json,sys,time,socket
from pathlib import Path
ROOTS=('/agibot/software/agent/bin/cfg/face_ui/','/agibot/software/agent_a/bin/cfg/face_ui/','/agibot/software/agent_b/bin/cfg/face_ui/','/home/agi/gdk_resources/collection_faces/')

def validate(r):
    kind=r.get('kind')
    if kind not in ('status','volume','speak','video','display_off','faces','audio'):raise ValueError('未知请求')
    if kind in ('volume','speak','audio') and (type(r.get('volume')) is not int or not 0<=r['volume']<=100):raise ValueError('音量范围为 0–100')
    if kind=='speak' and (not isinstance(r.get('text'),str) or not 1<=len(r['text'].strip())<=1000):raise ValueError('请输入 1–1000 字')
    if kind=='video':
        p=r.get('path','')
        if not isinstance(p,str) or not p.startswith(ROOTS) or '..' in p.split('/') or not p.endswith('.mp4'):raise ValueError('表情路径无效')
        resolved=Path(p).resolve()
        if not str(resolved).startswith(ROOTS) or not resolved.is_file():raise ValueError('交互板找不到表情文件')
        if r.get('loops') not in (1,-1):raise ValueError('循环参数无效')
    if kind=='audio':
        p=r.get('path','');root='/home/agi/gdk_resources/collection_audio/'
        if not isinstance(p,str) or not p.startswith(root) or '..' in p.split('/') or not p.endswith('.wav'):raise ValueError('音频路径无效')
        resolved=Path(p).resolve()
        if not str(resolved).startswith(root) or not resolved.is_file():raise ValueError('交互板找不到音频文件')

def player_path():
    try:
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(1);s.connect('/tmp/mpv-socket');s.sendall(b'{"command":["get_property","path"]}\n')
            return json.loads(s.recv(8192)).get('data')
    except (OSError,ValueError):return None

def execute(r):
    validate(r);kind=r['kind']
    if kind=='faces':
        p=Path('/agibot/software/agent/bin/cfg/face_ui')
        files=[str(x) for x in sorted(p.glob('*.mp4')) if x.is_file()]
        if not files:raise RuntimeError('未找到当前固件的内置表情')
        return {'faces':files}
    import rclpy
    from aimdk_msgs import srv
    rclpy.init();node=rclpy.create_node('g2_gui_media_'+str(__import__('os').getpid()))
    def call(name,fill=None):
        cls=getattr(srv,name);client=node.create_client(cls,'/aimdk_5Fmsgs/srv/'+name)
        try:
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError(name+' 服务不可用')
            req=cls.Request()
            if fill:fill(req)
            future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=8)
            if not future.done():raise RuntimeError(name+' 响应超时，执行结果未知，请勿连续重复提交')
            response=future.result()
            if hasattr(response,'result') and response.result!=0:raise RuntimeError(f'{name} 返回错误 {response.result}')
            header=getattr(response,'reponse',getattr(response,'header',None))
            if header is not None and hasattr(header,'status') and header.status.value not in (0,1):raise RuntimeError(getattr(header,'message','服务失败') or f'{name} 状态 {header.status.value}')
            return response
        finally:node.destroy_client(client)
    try:
        if kind=='status':return {'volume':int(call('GetVolume').audio_volume)}
        if kind in ('volume','speak','audio'):
            result=call('SetVolume',lambda q:setattr(q,'audio_volume',r['volume']))
            if result.audio_volume!=r['volume']:raise RuntimeError('返回音量与设置不一致')
        if kind=='speak':
            def fill(q):
                q.tts_req.text=r['text'].strip();q.tts_req.domain='g2_collection_gui';q.tts_req.trace_id=__import__('uuid').uuid4().hex
                q.tts_req.priority_level.value=6;q.tts_req.is_interrupted=True
            result=call('PlayTts',fill)
            if not result.tts_resp.is_success:raise RuntimeError(result.tts_resp.error_message or '播报被拒绝')
        if kind=='audio':
            def fill(q):
                p=Path(r['path']);q.file.pkg_name='g2_collection_gui'
                q.file.file_name=p.name;q.file.file_path=str(p.parent)+'/'
                q.file.priority=6
            call('PlayAudioFile',fill)
        if kind=='video':
            call('SwitchDisplay',lambda q:setattr(q,'display_enabled',True))
            def fill(q):
                q.video_display.video_path=r['path'];q.video_display.loop=0 if r['loops']==-1 else 1
                q.requester='g2_collection_gui';q.mode=0
            call('VideoDisplayReq',fill)
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                path=player_path()
                if path and Path(path).resolve()==Path(r['path']).resolve():return {'accepted':True,'confirmed':True}
                time.sleep(.2)
            return {'accepted':True,'confirmed':False}
        if kind=='display_off':call('SwitchDisplay',lambda q:setattr(q,'display_enabled',False))
        return {'accepted':True}
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    try:print('G2_MEDIA_RESULT='+json.dumps({'ok':True,**execute(json.loads(sys.stdin.readline()))},ensure_ascii=False))
    except Exception as e:print('G2_MEDIA_RESULT='+json.dumps({'ok':False,'error':str(e)},ensure_ascii=False));sys.exit(1)
