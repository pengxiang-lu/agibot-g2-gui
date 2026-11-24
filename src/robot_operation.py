"""Qt manual-operation page. Commands require explicit enablement and focus."""
import json,math,shlex,time
from pathlib import Path
from PyQt5 import QtCore,QtGui,QtWidgets
from camera_preview import CameraPreview
from robot_interaction import InteractionPanel
HERE=Path(__file__).resolve().parent

class Joystick(QtWidgets.QWidget):
    changed=QtCore.pyqtSignal(float,float)
    def __init__(self,parent=None):
        super().__init__(parent);self.x=self.y=0.;self.drag=False;self.setMinimumSize(270,270);self.setMaximumSize(380,380);self.setFocusPolicy(QtCore.Qt.StrongFocus)
    def point(self,event):
        radius=min(self.width(),self.height())*.38
        x=(event.pos().x()-self.width()/2)/radius;y=(self.height()/2-event.pos().y())/radius
        r=max(1.,math.hypot(x,y));self.x=x/r;self.y=y/r;self.changed.emit(self.x,self.y);self.update()
    def mousePressEvent(self,event):
        if event.button()==QtCore.Qt.LeftButton and self.isEnabled():self.setFocus();self.drag=True;self.point(event)
    def mouseMoveEvent(self,event):
        if self.drag:self.point(event)
    def mouseReleaseEvent(self,event):self.release()
    def release(self):
        self.drag=False;self.x=self.y=0.;self.changed.emit(0.,0.);self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        c=QtCore.QPointF(self.width()/2,self.height()/2);r=min(self.width(),self.height())*.43
        p.setPen(QtGui.QPen(QtGui.QColor('#cee0ee'),2));p.setBrush(QtGui.QColor('#e7f2fa'));p.drawEllipse(c,r,r)
        p.setPen(QtGui.QPen(QtGui.QColor('#b8d3e7'),1));p.drawLine(QtCore.QPointF(c.x()-r*.85,c.y()),QtCore.QPointF(c.x()+r*.85,c.y()));p.drawLine(QtCore.QPointF(c.x(),c.y()-r*.85),QtCore.QPointF(c.x(),c.y()+r*.85))
        p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#d3e5f3'));p.drawEllipse(c,r*.18,r*.18)
        p.setPen(QtGui.QPen(QtGui.QColor('#2d8bc8'),3));p.setBrush(QtGui.QColor('#ffffff' if self.isEnabled() else '#e1e8ee'))
        knob=c+QtCore.QPointF(self.x*r*.88,-self.y*r*.88);p.drawEllipse(knob,27,27)
        p.setPen(QtGui.QColor('#47738d'));p.drawText(QtCore.QRectF(0,8,self.width(),25),QtCore.Qt.AlignCenter,'前进')
        p.drawText(QtCore.QRectF(0,self.height()-32,self.width(),25),QtCore.Qt.AlignCenter,'后退')

class OperationPage(QtWidgets.QWidget):
    def __init__(self,owner):
        super().__init__();self.owner=owner;self.proc=None;self.host=None;self.buffer=b'';self.armed=False;self.ready=False;self.feedback_at=0.;self.command={'kind':'idle'};self.counter=0;self.held_keys=set();self.stopping=False;self.arm_dialog=None;self.arm_pending_at=None
        layout=QtWidgets.QVBoxLayout(self)
        bar=QtWidgets.QHBoxLayout();self.message=QtWidgets.QLabel('请先连接机器人 SSH');self.message.setWordWrap(True);bar.addWidget(self.message,1)
        self.enable=QtWidgets.QPushButton('启用手动控制');self.enable.clicked.connect(self.arm);bar.addWidget(self.enable)
        stop=QtWidgets.QPushButton('停止操作');stop.setObjectName('stop');stop.clicked.connect(self.disarm);bar.addWidget(stop);layout.addLayout(bar)
        scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);layout.addWidget(scroll)
        content=QtWidgets.QWidget();body=QtWidgets.QVBoxLayout(content);scroll.setWidget(content)
        top=QtWidgets.QHBoxLayout();body.addLayout(top)
        base=QtWidgets.QGroupBox('底盘控制');base.setMaximumHeight(490);bv=QtWidgets.QVBoxLayout(base);self.joystick=Joystick();self.joystick.changed.connect(self.base_move);bv.addWidget(self.joystick,0,QtCore.Qt.AlignCenter)
        self.mode=QtWidgets.QComboBox();self.mode.addItem('阿克曼','ackermann');self.mode.addItem('蟹行','crab');self.mode.currentIndexChanged.connect(self.mode_changed);bv.addWidget(self.mode)
        self.hint=QtWidgets.QLabel('上下前后，左右转向；横向拖动为原地转向');self.hint.setWordWrap(True);bv.addWidget(self.hint)
        self.speed=QtWidgets.QComboBox()
        for v in (20,40,60,80,100):self.speed.addItem(f'速度 {v}%',v/100)
        self.speed.currentIndexChanged.connect(self.mode_changed);bv.addWidget(self.speed)
        note=QtWidgets.QLabel('拖动摇杆或按住方向键，松开停止。\n上限：0.25 m/s · 0.40 rad/s');note.setWordWrap(True);bv.addWidget(note);top.addWidget(base,1)
        self.camera=CameraPreview([('head_color','头部彩色相机')],self,fps=30,width=640);self.camera.setTitle('头部视角');self.camera.setMaximumHeight(490);self.camera.layout().setStretch(0,0);self.camera.layout().setStretch(1,1);top.addWidget(self.camera,2)
        self.interaction=InteractionPanel(owner,self);body.addWidget(self.interaction)
        upper=QtWidgets.QGroupBox('上身关节控制');uv=QtWidgets.QVBoxLayout(upper);body.addWidget(upper)
        opts=QtWidgets.QHBoxLayout();opts.addWidget(QtWidgets.QLabel('单次点动步长'))
        self.step=QtWidgets.QComboBox()
        for v in (1,2,5,10):self.step.addItem(f'{v}°',v)
        opts.addWidget(self.step);opts.addStretch();reset=QtWidgets.QPushButton('机器人复位');reset.clicked.connect(self.reset_robot);opts.addWidget(reset);uv.addLayout(opts)
        self.groups=QtWidgets.QTabWidget();uv.addWidget(self.groups);self.values={};self.joint_buttons=[]
        groups=[('头部',[(f'idx{i:02d}_head_joint{i-10}',f'头部关节 {i-10}') for i in range(11,14)]),('躯干',[(f'idx{i:02d}_body_joint{i}',f'躯干关节 {i}') for i in range(1,6)]),('左臂',[(f'idx{i:02d}_arm_l_joint{i-20}',f'左臂关节 {i-20}') for i in range(21,28)]),('右臂',[(f'idx{i:02d}_arm_r_joint{i-60}',f'右臂关节 {i-60}') for i in range(61,68)])]
        for title,joints in groups:
            page=QtWidgets.QWidget();grid=QtWidgets.QGridLayout(page);grid.setAlignment(QtCore.Qt.AlignTop);grid.setVerticalSpacing(10)
            for row,(name,label) in enumerate(joints):
                text=QtWidgets.QLabel(label);text.setToolTip(name);grid.addWidget(text,row,0)
                value=QtWidgets.QLabel('— °');grid.addWidget(value,row,1);self.values[name]=value
                for col,sign in ((2,-1),(3,1)):
                    b=QtWidgets.QPushButton('−' if sign<0 else '+');b.clicked.connect(lambda checked=False,n=name,d=sign:self.jog(n,d));grid.addWidget(b,row,col);self.joint_buttons.append(b)
            grid.setColumnStretch(1,1);self.groups.addTab(page,title)
        note=QtWidgets.QLabel('显示实测角度；点击 ± 低速移动一个步长。停止操作、切换分页或窗口失焦会中止点动。');note.setWordWrap(True);uv.addWidget(note)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.pulse);self.timer.start(50)
        QtWidgets.QApplication.instance().applicationStateChanged.connect(self.app_state)
        QtWidgets.QApplication.instance().installEventFilter(self)
    ARROWS={QtCore.Qt.Key_Up,QtCore.Qt.Key_Down,QtCore.Qt.Key_Left,QtCore.Qt.Key_Right}
    def clear_keys(self):
        if self.held_keys:
            self.held_keys.clear();self.joystick.release()
    def keyboard_allowed(self):
        app=QtWidgets.QApplication.instance();focus=app.focusWidget()
        return (self.ready and self.armed and self.owner.pages.currentIndex()==1
                and self.window().isActiveWindow() and not self.busy()
                and app.activeModalWidget() is None and app.activePopupWidget() is None
                and focus is not None and (focus is self or self.isAncestorOf(focus))
                and not (hasattr(self,'interaction') and (focus is self.interaction or self.interaction.isAncestorOf(focus)))
                and not isinstance(focus,(QtWidgets.QLineEdit,QtWidgets.QTextEdit,
                    QtWidgets.QPlainTextEdit,QtWidgets.QAbstractSpinBox,QtWidgets.QAbstractSlider,QtWidgets.QComboBox)))
    def eventFilter(self,obj,event):
        kind=event.type()
        if kind in (QtCore.QEvent.MouseButtonPress,QtCore.QEvent.FocusOut,
                    QtCore.QEvent.WindowDeactivate,QtCore.QEvent.ApplicationDeactivate):
            self.clear_keys()
        if kind not in (QtCore.QEvent.KeyPress,QtCore.QEvent.KeyRelease,QtCore.QEvent.ShortcutOverride):
            return super().eventFilter(obj,event)
        key=event.key()
        if key not in self.ARROWS:return super().eventFilter(obj,event)
        allowed=self.keyboard_allowed() and not self.joystick.drag
        if kind==QtCore.QEvent.ShortcutOverride:
            if allowed:event.accept();return True
            return False
        if kind==QtCore.QEvent.KeyRelease and key in self.held_keys:
            if not event.isAutoRepeat():
                self.held_keys.discard(key);self.keyboard_move()
            return True
        if not allowed:return False
        if event.isAutoRepeat():return True
        if kind==QtCore.QEvent.KeyPress:
            self.held_keys.add(key);self.keyboard_move()
        return True
    def keyboard_move(self):
        x=float((QtCore.Qt.Key_Right in self.held_keys)-(QtCore.Qt.Key_Left in self.held_keys))
        y=float((QtCore.Qt.Key_Up in self.held_keys)-(QtCore.Qt.Key_Down in self.held_keys))
        scale=max(1.,math.hypot(x,y));x/=scale;y/=scale
        self.joystick.x=x;self.joystick.y=y;self.joystick.update();self.base_move(x,y)
    def motion_busy(self):return self.owner.reset_busy or any(getattr(w,'motion',None) is not None for w in self.owner.replay_windows)
    def busy(self):return (hasattr(self,'interaction') and self.interaction.working) or self.motion_busy()
    def arm(self):
        if not self.ready or self.busy() or self.stopping or self.arm_dialog or self.arm_pending_at:return
        # Modeless confirmation: the stop button must remain clickable.
        box=QtWidgets.QMessageBox(QtWidgets.QMessageBox.Question,'启用手动控制',
            '请停止 Genie Link 遥操作、导航和其他运动控制程序，确认运动范围安全。是否启用电脑手动控制？',
            QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,self)
        box.setWindowModality(QtCore.Qt.NonModal);box.setDefaultButton(QtWidgets.QMessageBox.No)
        self.arm_dialog=box;self.message.setText('等待确认启用手动控制；可随时点击停止操作')
        def confirmed(result):
            current=self.arm_dialog is box
            if current:self.arm_dialog=None
            box.deleteLater()
            if current and result==QtWidgets.QMessageBox.Yes and self.ready and not self.busy() and not self.stopping and self.owner.pages.currentIndex()==1:
                self.arm_pending_at=time.monotonic();self.command={'kind':'arm'}
                self.message.setText('正在启用手动控制…');self.send(self.command)
        box.finished.connect(confirmed);box.show()
    def send(self,msg):
        if not self.stopping and self.proc and self.proc.state()==QtCore.QProcess.Running:
            if self.proc.bytesToWrite()>8192:self.shutdown();return
            self.proc.write((json.dumps(msg)+'\n').encode())
    def stop_reason(self,text):
        self.disarm();self.message.setText(text)
        if hasattr(self.owner,'append'):self.owner.append('机器人操作：'+text)
    def disarm(self):
        self.arm_pending_at=None
        if self.arm_dialog:
            box=self.arm_dialog;self.arm_dialog=None;box.reject()
        self.held_keys.clear();self.armed=False;self.message.setText('手动控制已停止');self.command={'kind':'stop'};self.joystick.release();self.command={'kind':'stop'};self.send(self.command)
    def app_state(self,state):
        if state!=QtCore.Qt.ApplicationActive and (self.armed or self.arm_pending_at is not None):self.stop_reason('操作已停止：应用失去前台焦点')
    def mode_changed(self):
        self.clear_keys();self.joystick.release();self.command={'kind':'idle'};self.send(self.command)
        self.hint.setText('上下前后，左右转向；横向拖动为原地转向' if self.mode.currentData()=='ackermann' else '上下前后，左右横移；斜向拖动为斜移')
    def base_move(self,x,y):
        if not self.armed:return
        self.command={'kind':'base','x':x,'y':y,'mode':self.mode.currentData(),'speed':self.speed.currentData()} if x or y else {'kind':'idle'}
        self.send(self.command)
    def jog(self,name,sign):
        if not self.armed:return
        self.clear_keys();self.joystick.release();self.counter+=1
        self.command={'kind':'joint','name':name,'delta':math.radians(self.step.currentData()*sign),'speed':self.speed.currentData(),'id':self.counter};self.send(self.command)
    def reset_robot(self):self.shutdown();self.owner.reset_robot()
    def connect_source(self,host):
        self.host=host;self.ready=False;self.buffer=b'';self.command={'kind':'idle'}
        self.proc=QtCore.QProcess(self);self.proc.readyReadStandardOutput.connect(self.read)
        self.proc.readyReadStandardError.connect(lambda:self.proc.readAllStandardError() if self.proc else None)
        self.proc.finished.connect(self.finished)
        self.proc.errorOccurred.connect(lambda error:self.message.setText('操作连接失败，请切换分页后重试'))
        code=(HERE/'operation_source.py').read_text();remote='source /home/agi/app/env.sh >/dev/null 2>&1 && exec python3 -u -c '+shlex.quote(code)
        self.proc.start('ssh',['-T',*self.owner.ssh_options(),'-o','BatchMode=yes','-o','ControlMaster=no',host,'bash -lc '+shlex.quote(remote)])
        self.message.setText('正在连接操作通道…')
    def read(self):
        if not self.proc:return
        self.buffer+=bytes(self.proc.readAllStandardOutput())
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            try:m=json.loads(line)
            except ValueError:continue
            kind=m.get('kind')
            if kind=='ready' and not self.stopping:self.ready=True;self.message.setText('已连接 · 手动控制未启用')
            elif kind=='armed':
                if self.arm_pending_at is None or self.stopping:
                    self.send({'kind':'stop'});continue
                self.arm_pending_at=None;self.armed=True;self.command={'kind':'idle'};self.message.setText('手动控制已启用 · 方向键 / 摇杆控制，松开停止');self.joystick.setFocus()
            elif kind=='state':
                self.feedback_at=time.monotonic()
                for n,v in m['positions'].items():
                    if n in self.values:self.values[n].setText(f'{math.degrees(v):.2f}°')
                if not m['healthy'] and (self.armed or self.arm_pending_at is not None):self.stop_reason('操作已停止：机器人状态不允许运动 · '+m.get('reason','请检查故障与模式'))
            elif kind=='done' and self.command.get('id')==m['id']:self.command={'kind':'idle'};self.message.setText('点动完成')
            elif kind=='error':self.stop_reason('操作已停止：'+m['message'])
    def finished(self,*args):
        self.ready=False;self.armed=False;self.clear_keys();self.command={'kind':'stop'}
        # Retain the stopped process until an explicit page reconnect, never auto-rearm.
    def pulse(self):
        if self.arm_pending_at is not None and time.monotonic()-self.arm_pending_at>5:
            self.disarm();self.message.setText('启用手动控制超时，已取消，请检查机器人状态后重试')
        host=self.owner.connected_host if self.owner.connected_host==self.owner.ssh.text().strip() else None
        visible=self.owner.pages.currentIndex()==1
        self.camera.configure(host if visible else None,self.owner.ssh_session.name,['head_color'])
        if 'head_color' in self.camera.views:self.camera.views['head_color'].setMinimumHeight(240)
        if not visible or not host or self.motion_busy():
            if self.proc:self.shutdown()
        elif self.proc is None:self.connect_source(host)
        elif self.host!=host:self.shutdown()
        if self.armed and time.monotonic()-self.feedback_at>.5:
            self.stop_reason(f'操作已停止：关节反馈超过 500 ms 未更新（{(time.monotonic()-self.feedback_at)*1000:.0f} ms）')
        elif self.armed and not self.window().isActiveWindow():self.stop_reason('操作已停止：操作窗口失去焦点')
        enabled=self.ready and not self.busy() and visible
        self.enable.setEnabled(enabled and not self.armed and not self.stopping and self.arm_pending_at is None and self.arm_dialog is None)
        self.joystick.setEnabled(enabled and self.armed)
        for b in self.joint_buttons:b.setEnabled(enabled and self.armed)
        self.send(self.command)
    def shutdown(self):
        if self.stopping:return
        self.stopping=True
        self.disarm();p=self.proc
        self.ready=False;self.armed=False
        if not p:
            self.stopping=False;return
        # Do not waitForFinished on the GUI thread. The robot command lease
        # expires independently if the SSH channel cannot deliver stop/quit.
        if p.state()==QtCore.QProcess.Running:
            p.write(b'{"kind":"stop"}\n{"kind":"quit"}\n');p.closeWriteChannel()
        def complete(*args):
            if self.proc is not p:return
            self.proc=None;self.stopping=False;p.deleteLater()
        def terminate():
            if self.proc is p and p.state()!=QtCore.QProcess.NotRunning:p.terminate()
        def kill():
            if self.proc is p and p.state()!=QtCore.QProcess.NotRunning:p.kill()
        p.finished.connect(complete)
        if p.state()==QtCore.QProcess.NotRunning:complete()
        else:
            QtCore.QTimer.singleShot(6000,terminate)
            QtCore.QTimer.singleShot(8000,kill)
    def close_all(self):
        if not self.interaction.shutdown():return False
        self.shutdown()
        if self.stopping:return False
        QtWidgets.QApplication.instance().removeEventFilter(self);self.timer.stop();return self.camera.shutdown()
