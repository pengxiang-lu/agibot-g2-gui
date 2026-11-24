"""Read-only side-by-side comparison of recorded and live camera streams."""
import cv2
import time
import json
import shlex
import numpy as np
import pyarrow.parquet as pq
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from PyQt5 import QtCore, QtGui, QtWidgets
from collect import Source, decode_images
from replay import VideoView, CAMERAS

class CameraWorker(QtCore.QThread):
    image = QtCore.pyqtSignal(str, QtGui.QImage)
    status = QtCore.pyqtSignal(str)
    def __init__(self, host, control_dir, parent=None):
        super().__init__(parent); self.host=host; self.control_dir=control_dir; self.source=None
    def run(self):
        args=SimpleNamespace(fps=10, timeout=25, max_sync_ms=100, max_action_age_ms=200,
            max_width=640, transport='jpeg', jpeg_quality=85, state_topic='/hal/joint_state',
            action_topic='/hal/joint_cmd', cameras=[x for x,_ in CAMERAS], simulate=False,
            local_gdk=False, ssh=self.host, ssh_control_dir=self.control_dir,
            require_ssh_control=True, known_hosts=None, remote_env='/home/agi/app/env.sh')
        try:
            self.source=Source(args)
            ready,_=self.source.receive(65)
            if self.isInterruptionRequested(): return
            self.source.start(); self.status.emit('实时画面已连接')
            while not self.isInterruptionRequested():
                meta,payloads=self.source.receive(5)
                if meta.get('kind')!='frame': break
                images=decode_images(meta,payloads,ready)
                for key,_ in CAMERAS:
                    image=images['observation.images.'+key];h,w=image.shape[:2]
                    self.image.emit(key,QtGui.QImage(image.data,w,h,int(image.strides[0]),QtGui.QImage.Format_RGB888).copy())
        except Exception as exc:
            if not self.isInterruptionRequested(): self.status.emit('实时画面连接失败：'+str(exc))
        finally:
            if self.source: self.source.close()
    def stop(self):
        self.requestInterruption()
        if self.source and self.source.process.poll() is None: self.source.process.terminate()

class CompareDialog(QtWidgets.QDialog):
    def __init__(self,path,host,control_dir,parent=None):
        super().__init__(parent); self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window | QtCore.Qt.WindowMinMaxButtonsHint | QtCore.Qt.WindowCloseButtonHint)
        self.setWindowTitle('真机回放');self.resize(1450,900)
        self.path=Path(path);self.host=host;self.control_dir=control_dir;self.motion=None;self.live_times={};self.motion_buffer=b''
        import json
        info=json.loads((Path(path)/'meta/info.json').read_text())
        self.fps=int(info['fps']); self.count=int(info['total_frames']);self.current=-1
        self.captures={}; self.recorded={}; self.live={}
        layout=QtWidgets.QVBoxLayout(self)
        note=QtWidgets.QLabel('真机回放控制双臂与夹爪：先低速移动至录制起点，再以所选速度回放。实时画面与采集画面存在传输延迟。');note.setWordWrap(True);layout.addWidget(note)
        from preflight import inspect_dataset
        try:
            report=inspect_dataset(path)
            summary=f"轨迹检查：{report['frames']} 帧 · {report['fps']:g} FPS · 双臂相邻帧最大速度 {report['max_arm_recorded_speed']:.3f} rad/s · 夹爪范围 {report['gripper_min']:.3f}～{report['gripper_max']:.3f} rad"
            if report['simulated']: summary+=' · 模拟数据，不能用于真机回放'
        except Exception as exc: summary='轨迹检查未通过：'+str(exc)
        preflight=QtWidgets.QLabel(summary);preflight.setWordWrap(True);layout.addWidget(preflight)
        camera_scroll=QtWidgets.QScrollArea();camera_scroll.setWidgetResizable(True)
        camera_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        camera_content=QtWidgets.QWidget();grid=QtWidgets.QGridLayout(camera_content)
        camera_scroll.setWidget(camera_content);layout.addWidget(camera_scroll,1)
        for i,(key,label) in enumerate(CAMERAS):
            for row,title,views in [(0,'采集记录',self.recorded),(1,'真机实时',self.live)]:
                box=QtWidgets.QGroupBox(title+' · '+label);v=QtWidgets.QVBoxLayout(box);view=VideoView();view.setMinimumSize(240,260);v.addWidget(view);views[key]=view;grid.addWidget(box,row,i)
            files=sorted((Path(path)/'videos'/('observation.images.'+key)).rglob('*.mp4'))
            if len(files)>1: raise ValueError(label+'有多个视频分块，暂不支持')
            if files:
                cap=cv2.VideoCapture(str(files[0]))
                if not cap.isOpened():raise ValueError('无法打开'+label)
                frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                if frames and frames!=self.count: raise ValueError(label+'视频帧数与记录不一致')
                self.captures[key]=cap
        controls=QtWidgets.QHBoxLayout();layout.addLayout(controls)
        self.start_motion=QtWidgets.QPushButton('准备真机回放');self.start_motion.clicked.connect(self.prepare_motion);controls.addWidget(self.start_motion)
        self.stop_motion=QtWidgets.QPushButton('停止真机动作');self.stop_motion.setEnabled(False);self.stop_motion.clicked.connect(self.stop_replay);controls.addWidget(self.stop_motion)
        self.play=QtWidgets.QPushButton('播放采集记录');self.play.clicked.connect(self.toggle);controls.addWidget(self.play)
        controls.addWidget(QtWidgets.QLabel('回放速度'))
        self.replay_speed=QtWidgets.QComboBox()
        for percent in range(10,101,10):self.replay_speed.addItem(f'{percent}%',percent/100.0)
        self.replay_speed.setCurrentIndex(9);controls.addWidget(self.replay_speed)
        self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.slider.setRange(0,self.count-1);self.slider.valueChanged.connect(self.show_frame);controls.addWidget(self.slider,1)
        self.progress=QtWidgets.QLabel();controls.addWidget(self.progress)
        self.status=QtWidgets.QLabel('正在连接真机相机…');layout.addWidget(self.status)
        self.timer=QtCore.QTimer(self);self.timer.setInterval(30);self.timer.timeout.connect(self.tick)
        self.heartbeat=QtCore.QTimer(self);self.heartbeat.setInterval(100);self.heartbeat.timeout.connect(self.motion_heartbeat)
        self.show_frame(0)
        self.worker=CameraWorker(host,control_dir,self);self.worker.image.connect(self.set_live);self.worker.status.connect(self.status.setText);self.worker.start()
    def set_live(self,key,image):
        self.live_times[key]=time.monotonic();self.live[key].set_image(image)
    def show_frame(self,index):
        for key,cap in self.captures.items():
            if index!=self.current+1:cap.set(cv2.CAP_PROP_POS_FRAMES,index)
            ok,frame=cap.read()
            if not ok:self.timer.stop();self.status.setText('视频解码失败：'+key);return
            frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB);h,w=frame.shape[:2]
            self.recorded[key].set_image(QtGui.QImage(frame.data,w,h,int(frame.strides[0]),QtGui.QImage.Format_RGB888).copy())
        self.current=index;self.progress.setText(f'{index+1} / {self.count} 帧')
        self.slider.blockSignals(True);self.slider.setValue(index);self.slider.blockSignals(False)
    def toggle(self):
        if self.timer.isActive():self.timer.stop();self.play.setText('播放采集记录');return
        if self.current>=self.count-1:self.show_frame(0)
        self.started=time.monotonic();self.start_frame=self.current;self.play.setText('暂停');self.timer.start()
    def tick(self):
        index=min(self.count-1,self.start_frame+int((time.monotonic()-self.started)*self.fps))
        if index!=self.current:self.show_frame(index)
        if index>=self.count-1:self.timer.stop();self.play.setText('播放采集记录')
    def prepare_motion(self):
        if getattr(self.parent(),'reset_busy',False):
            QtWidgets.QMessageBox.information(self,'真机回放','正在发送复位请求，请稍候并确认复位完成。');return
        if self.motion is not None:return
        if len(self.live_times)!=3 or any(time.monotonic()-t>2 for t in self.live_times.values()):
            QtWidgets.QMessageBox.warning(self,'真机回放','请先等待三路真机相机正常显示。');return
        try:
            if len(self.captures)!=3:raise ValueError('记录必须包含头部与左右手三路相机')
            from preflight import inspect_dataset, ARM_NAMES, GRIPPER_NAMES
            report=inspect_dataset(self.path)
            if report['simulated']:raise ValueError('模拟数据不能用于真机回放')
            manifest=json.loads((self.path/'capture_manifest.json').read_text())
            names=manifest['schema']['joint_names']
            rows=[]
            for f in sorted((self.path/'data').rglob('*.parquet')):
                rows.extend(pq.read_table(f,columns=['action','observation.gripper','frame_index','episode_index']).to_pylist())
            rows.sort(key=lambda row:(row['episode_index'],row['frame_index']))
            indexes=[names.index(n) for n in ARM_NAMES]
            samples=[[float(row['action'][i]) for i in indexes]+list(map(float,row['observation.gripper'])) for row in rows]
            joint_names=ARM_NAMES+GRIPPER_NAMES
            urdf=Path(__file__).resolve().parent.parent/'assets/genie_robot_description/urdf/G2_t2v2_crs_omnipicker.urdf'
            joints={joint.get('name'):joint for joint in ET.parse(urdf).getroot().findall('joint')}
            velocity_limits=[]
            for name in joint_names:
                joint=joints.get(name); limit=joint.find('limit') if joint is not None else None
                if limit is None or limit.get('velocity') is None:raise ValueError('模型缺少关节速度限值：'+name)
                velocity_limits.append(float(limit.get('velocity')))
            config={'names':joint_names,'samples':samples,'fps':self.fps,'velocity_limits':velocity_limits,'speed_scale':float(self.replay_speed.currentData())}
            self.motion_buffer=b'';self.motion=QtCore.QProcess(self)
            self.motion.setProcessChannelMode(QtCore.QProcess.SeparateChannels)
            self.motion.readyReadStandardOutput.connect(self.motion_output)
            self.motion.readyReadStandardError.connect(self.motion_errors)
            self.motion.finished.connect(self.motion_finished)
            remote_script=(Path(__file__).parent/'robot_servo.py').read_text()
            remote='source /home/agi/app/env.sh >/dev/null 2>&1 && exec python3 -u -c '+shlex.quote(remote_script)
            args=['-T','-o','BatchMode=yes','-o','ControlMaster=no','-o','ControlPath='+str(Path(self.control_dir)/'%C'),'-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2',self.host,'bash -lc '+shlex.quote(remote)]
            self.motion.start('ssh',args)
            if not self.motion.waitForStarted(3000):raise RuntimeError('无法启动 SSH 控制进程')
            self.motion.write((json.dumps(config,allow_nan=False,separators=(',',':'))+'\n').encode())
            self.replay_speed.setEnabled(False);self.start_motion.setEnabled(False);self.stop_motion.setEnabled(True);self.play.setEnabled(False);self.slider.setEnabled(False)
            self.timer.stop();self.heartbeat.start();self.status.setText('正在检查真机关节限位和起始姿态…')
        except Exception as exc:
            if self.motion:self.motion.kill();self.motion=None
            QtWidgets.QMessageBox.warning(self,'无法准备真机回放',str(exc))
    def motion_heartbeat(self):
        if not self.motion:return
        if len(self.live_times)!=3 or any(time.monotonic()-t>2 for t in self.live_times.values()):
            self.status.setText('真机相机数据中断，已请求停止');self.stop_replay();return
        self.motion.write(b'PING\n')
    def motion_output(self):
        if not self.motion:return
        self.motion_buffer+=bytes(self.motion.readAllStandardOutput())
        while b'\n' in self.motion_buffer:
            line,self.motion_buffer=self.motion_buffer.split(b'\n',1)
            try:msg=json.loads(line)
            except Exception:continue
            kind=msg.get('kind')
            if kind in ('ready','transition','started','tracking_error','error','stopped','finished'):
                try:
                    log_dir=Path(__file__).resolve().parent.parent/'logs'
                    log_dir.mkdir(exist_ok=True)
                    with (log_dir/'robot_replay.jsonl').open('a',encoding='utf-8') as log:
                        log.write(json.dumps({'time':time.strftime('%Y-%m-%d %H:%M:%S'),
                                              'dataset':str(self.path),'event':msg},ensure_ascii=False)+'\n')
                except OSError as exc:
                    print('无法保存回放诊断日志：'+str(exc),flush=True)
            if kind=='ready':
                text=(f"即将先将双臂和夹爪低速移动到录制起点，再以 {msg['scale']:.1%} 的速度回放，预计 {msg['duration']:.1f} 秒。\n"
                      f"当前最大起点偏差 {msg['max_start_delta']:.3f} rad。\n请确认机器人周围无人员和障碍、无其他控制程序，且急停可用。是否开始？")
                speed_limited=msg.get('speed_limited',[])
                if speed_limited:
                    details='；'.join(f"{item['joint']}：{item['recorded']:.3f} / {item['limit']:.3f} rad/s" for item in speed_limited)
                    text+='\n\n录制速度超过模型限值，将按上述比例降速：\n'+details
                corrections=msg.get('limit_corrections',[])
                if corrections:
                    details='；'.join(f"{item['joint']}：{item['frames']} 帧，最大 {item['max_rad']:.6f} rad" for item in corrections)
                    text+='\n\n轨迹有轻微越限，回放时会限制到机器人允许范围（不修改原始数据）：\n'+details
                if QtWidgets.QMessageBox.question(self,'确认真机动作',text,QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)==QtWidgets.QMessageBox.Yes:
                    if self.motion:self.motion.write(b'START\n')
                else:self.stop_replay()
            elif kind=='transition':self.status.setText('正在低速移动到初始位姿…')
            elif kind=='started':self.status.setText('正在回放真机动作')
            elif kind=='progress':self.show_frame(int(msg['frame']))
            elif kind=='finished':self.status.setText('真机动作回放结束')
            elif kind=='stopped':self.status.setText('真机回放停止：'+msg.get('reason',''))
            elif kind=='error':
                message='真机回放失败：'+msg.get('message','')
                self.status.setText(message);self.status.setToolTip(message)
                QtWidgets.QMessageBox.warning(self,'真机回放已停止',message)
    def motion_errors(self):
        if self.motion:
            data=bytes(self.motion.readAllStandardError()).decode(errors='replace').strip()
            if data:print(data,flush=True)
    def stop_replay(self):
        self.heartbeat.stop()
        proc=self.motion
        if proc and proc.state()!=QtCore.QProcess.NotRunning:
            proc.write(b'STOP\n');proc.closeWriteChannel()
            if not proc.waitForFinished(2500):proc.kill();proc.waitForFinished(1000)
    def motion_finished(self,*args):
        self.heartbeat.stop();self.replay_speed.setEnabled(True);self.start_motion.setEnabled(True);self.stop_motion.setEnabled(False);self.play.setEnabled(True);self.slider.setEnabled(True)
        old=self.motion;self.motion=None
        if old:old.deleteLater()

    def closeEvent(self,event):
        self.timer.stop();self.stop_replay();self.worker.stop()
        if not self.worker.wait(8000):
            event.ignore();self.status.setText('正在关闭相机连接，请稍候…');return
        for cap in self.captures.values():cap.release()
        event.accept()
