"""Read-only VR connection and left-controller button stream, run on robot."""
import glob,json,os,sys,time,threading
out=os.fdopen(os.dup(1),'w',buffering=1);os.dup2(2,1)

def emit(data):
    print(json.dumps(data,ensure_ascii=False,separators=(',',':')),file=out,flush=True)

def usb_connected():
    for path in glob.glob('/sys/bus/usb/devices/*'):
        try:
            with open(path+'/idVendor') as f:vendor=f.read().strip().lower()
            with open(path+'/idProduct') as f:product=f.read().strip().lower()
            if (vendor,product) in {('2d40','00b7'),('05c6','9024')}:return True
        except (OSError,UnicodeError):continue
    return False

def main():
    import agibot_gdk as g
    from genie_msgs_pb.msg.VRNetwork_pb2 import VRNetwork
    from rh_msgs_v4_pb.robot_frame_data_pb2 import RobotFrameData
    if g.gdk_init()!=g.GDKRes.kSuccess:raise RuntimeError('GDK 初始化失败')
    node=g.dds.Node('vr_buttons_gui_%d'%os.getpid())
    qos=g.dds.GDKQoS(depth=64,reliability=g.dds.GDKQoSReliability.BEST_EFFORT,durability=g.dds.GDKQoSDurability.VOLATILE)
    lock=threading.Lock();state={'network':None,'network_at':0.,'keys_at':0.,'previous':None,'armed':False}
    events=[];last_press={'X':0.,'Y':0.}
    def network(message,info):
        with lock:
            state['network']=bool(message.is_connected);state['network_at']=time.monotonic()
            if not state['network']:state['armed']=False;state['previous']=None
    def keys(message,info):
        now=time.monotonic()
        for device in message.device_inputs:
            if int(device.device_type)!=0:continue
            current=(bool(device.key_one),bool(device.key_two))
            with lock:
                if now-state['keys_at']>2:state['armed']=False;state['previous']=None
                state['keys_at']=now
                network_ok=state['network'] is True and now-state['network_at']<=5
                if not network_ok:state['armed']=False;state['previous']=None;return
                if not state['armed']:
                    if current==(False,False):state['armed']=True;state['previous']=current
                    return
                prev=state['previous'];state['previous']=current
                key='X' if current==(True,False) and prev==(False,False) else ('Y' if current==(False,True) and prev==(False,False) else None)
                if key and now-last_press[key]>=.3:
                    last_press[key]=now;events.append(key)
            return
    subs=[node.create_subscriber('/remote/vr_network',VRNetwork,qos,network),
          node.create_subscriber('/remote/vr_data',RobotFrameData,qos,keys)]
    last_status=0.
    try:
        while True:
            now=time.monotonic()
            with lock:
                emitted=events[:];events.clear()
                net=state['network'] if now-state['network_at']<=5 else None
                data=now-state['keys_at']<=2
                if net is not True or not data:state['armed']=False;state['previous']=None
            if net is True and data:
                for key in emitted:emit({'kind':'key','key':key})
            if now-last_status>=1:
                emit({'kind':'status','usb_connected':usb_connected(),'vr_connected':net,'key_stream':data})
                last_status=now
            time.sleep(.04)
    except (BrokenPipeError,KeyboardInterrupt):pass
    finally:g.gdk_release()

if __name__=='__main__':
    try:main()
    except Exception as exc:
        try:emit({'kind':'error','message':str(exc)})
        except BrokenPipeError:pass
        sys.exit(1)
