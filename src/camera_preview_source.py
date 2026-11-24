"""Rate-limited camera preview; enables selected camera publications, never switches robot mode."""
import json,os,sys,threading,time
from gdk_source import CAMERAS,rgb_image,send

def enable_selected(camera,names,config_path='/home/agi/app/gdk/config/thor_camera_conf.json'):
    import fcntl,tempfile
    from pathlib import Path
    path=Path(config_path)
    # Serialize local GUI instances, preserving every unselected camera and FPS.
    with open(str(path)+'.collection.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        original=path.read_text();config=json.loads(original)
        entries={v['name']:v for v in config.values() if isinstance(v,dict) and 'name' in v}
        missing=set(names)-entries.keys()
        if missing:raise RuntimeError('相机配置缺少：'+', '.join(sorted(missing)))
        changed=[n for n in names if entries[n].get('publish') is not True]
        if not changed:return []
        for name in changed:entries[name]['publish']=True
        backup=path.with_name(path.name+'.before_collection_'+str(time.time_ns()))
        backup.write_text(original)
        def replace(text):
            fd,tmp=tempfile.mkstemp(prefix=path.name+'.',dir=path.parent)
            try:
                with os.fdopen(fd,'w') as f:f.write(text)
                os.chmod(tmp,path.stat().st_mode & 0o777)
                os.replace(tmp,path)
            finally:
                if os.path.exists(tmp):os.unlink(tmp)
        replace(json.dumps(config,ensure_ascii=False,indent=4)+'\n')
        try:camera.set_dev_camera_config(str(path))
        except Exception:
            replace(original)
            raise RuntimeError('开启相机发布失败，配置文件已还原；请检查 GDK 相机配置服务')
        return changed

def main():
    out=os.fdopen(os.dup(1),'wb',buffering=0);os.dup2(2,1)
    import cv2
    import agibot_gdk as g
    cv2.setNumThreads(1)
    names=json.loads(sys.argv[1]);stop=threading.Event();ack=threading.Event();camera=None
    fps=max(1,min(30,int(sys.argv[2]))) if len(sys.argv)>2 else 5
    width=max(160,min(1280,int(sys.argv[3]))) if len(sys.argv)>3 else 480
    if not names or any(n not in CAMERAS for n in names):raise ValueError('相机选择无效')
    def watch():
        if fps<=5:
            sys.stdin.buffer.read(1);stop.set();return
        while not stop.is_set():
            line=sys.stdin.buffer.readline()
            if not line:stop.set();ack.set();return
            if line.strip()==b'ACK':ack.set()
    threading.Thread(target=watch,daemon=True).start()
    try:
        if g.gdk_init()!=g.GDKRes.kSuccess:raise RuntimeError('GDK 初始化失败')
        types={n:getattr(g.CameraType,CAMERAS[n]) for n in names};camera=g.Camera(list(types.values()))
        changed=enable_selected(camera,names)
        if changed:send(out,{'kind':'config','message':'已提交相机开启配置：'+', '.join(changed)+'；若无画面，请停止运动后重新切换 develop 模式'})
        last={}
        while not stop.is_set():
            begin=time.monotonic()
            for name,t in types.items():
                if stop.is_set():break
                try:
                    im=camera.get_latest_image(t,150.0)
                    stamp=int(im.timestamp_ns)
                    if stamp<=0 or stamp==last.get(name):continue
                    rgb=rgb_image(im);h,w=rgb.shape[:2]
                    if w>width:rgb=cv2.resize(rgb,(width,max(1,round(h*width/w))))
                    ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,75])
                    if not ok:raise RuntimeError('图像编码失败')
                    ack.clear()
                    send(out,{'kind':'image','camera':name,'width':w,'height':h,'timestamp_ns':stamp},[encoded.tobytes()]);last[name]=stamp
                    # At most one frame in flight: never queue seconds of stale video.
                    if fps>5 and not ack.wait(2):
                        stop.set();raise BrokenPipeError('Preview acknowledgement timed out')
                except BrokenPipeError:raise
                except Exception as exc:send(out,{'kind':'camera_error','camera':name,'message':'未收到相机图像；已确保配置开启。新开启相机需停止运动后重新切换 develop 模式。详情：'+str(exc)})
            stop.wait(max(0,1/fps-(time.monotonic()-begin)))
    except BrokenPipeError:pass
    except Exception as exc:send(out,{'kind':'error','message':str(exc)})
    finally:
        if camera is not None:camera.close_camera()
        g.gdk_release()

if __name__=='__main__':main()
