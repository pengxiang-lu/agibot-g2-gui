"""Selected-camera preview with automatic publication enablement."""
import json,shlex,subprocess,threading,time
from pathlib import Path
from PyQt5 import QtCore,QtGui,QtWidgets
from collect import read_packet
HERE=Path(__file__).resolve().parent

class PreviewWorker(QtCore.QThread):
    def __init__(self,host,control_dir,names,parent=None,fps=5,width=480):
        super().__init__(parent);self.host=host;self.control_dir=control_dir;self.names=names;self.fps=fps;self.width=width
        self.proc=None;self.lock=threading.Lock();self.latest={};self.arrivals={};self.error='';self.config_message=''
    def run(self):
        try:
            # Embed the existing read-only image decoder without running its collector.
            helper=(HERE/'gdk_source.py').read_text()
            code="import types,sys\nm=types.ModuleType('gdk_source');sys.modules['gdk_source']=m\nexec("+repr(helper)+",m.__dict__)\n"+(HERE/'camera_preview_source.py').read_text()
            remote='source /home/agi/app/env.sh >/dev/null 2>&1 && exec python3 -u -c '+shlex.quote(code)+' '+shlex.quote(json.dumps(self.names))+' '+str(self.fps)+' '+str(self.width)
            self.proc=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ControlMaster=no','-o','ControlPath='+str(Path(self.control_dir)/'%C'),
                '-o','ConnectTimeout=5','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2',self.host,'bash -lc '+shlex.quote(remote)],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
            if self.isInterruptionRequested():return
            while not self.isInterruptionRequested():
                meta,payloads=read_packet(self.proc.stdout)
                if meta['kind']=='error':raise RuntimeError(meta['message'])
                if meta['kind']=='config':self.config_message=meta['message']
                if meta['kind']=='image':
                    image=QtGui.QImage.fromData(payloads[0],'JPG')
                    if image.isNull():raise RuntimeError('预览图像解码失败')
                    now=time.monotonic();name=meta['camera']
                    arrivals=self.arrivals.setdefault(name,[])
                    arrivals.append(now);arrivals[:]=[t for t in arrivals if now-t<=2]
                    meta['received_fps']=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0
                    with self.lock:self.latest[name]=(image,meta,now)
                    if self.fps>5:
                        self.proc.stdin.write(b'ACK\n');self.proc.stdin.flush()
                elif meta['kind']=='camera_error':
                    with self.lock:self.latest[meta['camera']]=(None,meta,time.monotonic())
        except Exception as exc:
            if not self.isInterruptionRequested():self.error=str(exc)
        finally:
            if self.proc:
                if self.proc.poll() is None:
                    self.proc.terminate()
                    try:self.proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
                for stream in (self.proc.stdin,self.proc.stdout):
                    if stream:stream.close()
    def stop(self):
        self.requestInterruption()
        if self.proc and self.proc.poll() is None:self.proc.terminate()

class ImageView(QtWidgets.QLabel):
    def __init__(self):
        super().__init__('等待图像');self.image=None;self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(200,150);self.setMaximumHeight(260)
        self.setStyleSheet('background:#eaf3fa;border-radius:6px;color:#52718c;')
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored,QtWidgets.QSizePolicy.Preferred)
    def show_image(self,image):self.image=image;self.render_image()
    def render_image(self):
        if self.image is not None:self.setPixmap(QtGui.QPixmap.fromImage(self.image).scaled(self.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
    def resizeEvent(self,event):super().resizeEvent(event);self.render_image()
    def placeholder(self,text):self.image=None;self.clear();self.setText(text)

class CameraPreview(QtWidgets.QGroupBox):
    def __init__(self,cameras,parent=None,*,fps=5,width=480):
        super().__init__('已选相机实时预览',parent);self.fps=max(1,min(30,int(fps)));self.width=max(160,min(1280,int(width)));self.cameras=dict(cameras);self.worker=None;self.target=None;self.active=None;self.retry_at=0
        self.views={};self.details={};self.seen={}
        layout=QtWidgets.QVBoxLayout(self);self.status=QtWidgets.QLabel('连接机器人 SSH 后显示画面');self.status.setWordWrap(True);layout.addWidget(self.status)
        self.grid=QtWidgets.QGridLayout();layout.addLayout(self.grid)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(max(10,round(1000/self.fps)))
    def configure(self,host,control_dir,names,simulate=False):
        target=(host,control_dir,tuple(names),simulate)
        if target==self.target:return
        self.target=target;self.retry_at=0
        if self.worker:self.worker.stop()
        while self.grid.count():
            item=self.grid.takeAt(0)
            if item.widget():item.widget().deleteLater()
        self.views={};self.details={};self.seen={}
        for i,name in enumerate(names):
            box=QtWidgets.QGroupBox(self.cameras[name]);v=QtWidgets.QVBoxLayout(box);view=ImageView();detail=QtWidgets.QLabel(name);detail.setWordWrap(True);detail.setStyleSheet('font-size:15px;')
            v.addWidget(view);v.addWidget(detail);self.grid.addWidget(box,i//3,i%3);self.views[name]=view;self.details[name]=detail
        self.status.setText('模拟模式不显示真机预览' if simulate else ('请勾选采集相机' if not names else ('正在连接相机…' if host else '连接机器人 SSH 后显示画面')))
    def tick(self):
        if self.worker and not self.worker.isRunning():
            if self.worker.error:self.status.setText('预览连接失败：'+self.worker.error)
            self.worker.deleteLater();self.worker=None;self.retry_at=time.monotonic()+5
        if not self.target:return
        host,control_dir,names,simulate=self.target
        if self.worker and self.active!=self.target:return
        if not self.worker:
            if not host or not names or simulate or time.monotonic()<self.retry_at:return
            self.active=self.target;self.worker=PreviewWorker(host,control_dir,names,self,fps=self.fps,width=self.width);self.worker.start();return
        with self.worker.lock:latest=dict(self.worker.latest)
        live=0
        for name,(image,meta,received) in latest.items():
            if name not in self.views:continue
            if image is None or time.monotonic()-received>2:
                self.views[name].placeholder('暂无实时画面');self.details[name].setText(meta.get('message','图像已超过 2 秒未更新'));continue
            live+=1
            if self.seen.get(name)!=received:
                self.views[name].show_image(image);self.details[name].setText(f"{meta['width']} × {meta['height']} · 实收 {meta.get('received_fps',0):.1f} FPS / 最高 {self.fps} FPS")
                self.seen[name]=received
        self.status.setText(f'实时预览 {live}/{len(names)} 路 · 勾选自动开启图像发布'+(' · '+self.worker.config_message if self.worker.config_message and live<len(names) else ''))
    def shutdown(self):
        self.target=None
        if self.worker:
            self.worker.stop()
            return self.worker.wait(4000)
        return True
