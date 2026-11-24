#!/usr/bin/env python3
"""Local G2 LeRobot recorder UI. The collection process remains the single writer."""
import datetime as dt
import json
import os
from pathlib import Path
import queue
import shutil
import shlex
import signal
import subprocess
import sys
import threading
import tempfile
import time
import uuid

HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parent
TASK_ROOT = Path(os.environ.get('G2_TASK_ROOT', PROJECT_ROOT / 'gui_tasks')).expanduser().resolve()
CAMERAS = [
    ('head_color', '头部彩色'), ('hand_left_color', '左手彩色'),
    ('hand_right_color', '右手彩色'), ('head_stereo_left', '头部双目左'),
    ('head_stereo_right', '头部双目右'), ('head_left_fisheye', '头部左鱼眼'),
    ('head_right_fisheye', '头部右鱼眼'), ('head_back_fisheye', '头部后鱼眼'),
]

def now():
    return dt.datetime.now().astimezone().isoformat(timespec='seconds')

def atomic_json(path, obj):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(tmp, path)

def create_task(name):
    name = name.strip()
    if not name or len(name) > 200:
        raise ValueError('任务名称不能为空，且不能超过 200 个字符。')
    TASK_ROOT.mkdir(parents=True, exist_ok=True)
    tid = uuid.uuid4().hex
    folder = TASK_ROOT / tid
    folder.mkdir()
    task = {'id': tid, 'name': name, 'created_at': now()}
    atomic_json(folder / 'task.json', task)
    return task

def task_list():
    if not TASK_ROOT.exists():
        return []
    found = []
    for p in TASK_ROOT.glob('*/task.json'):
        try:
            data = json.loads(p.read_text(encoding='utf-8'))
            if data.get('id') == p.parent.name and isinstance(data.get('name'), str):
                found.append(data)
        except (OSError, ValueError):
            continue
    return sorted(found, key=lambda x: x.get('created_at', ''), reverse=True)

def rename_task(tid, name):
    name = name.strip()
    if not name or len(name) > 200:
        raise ValueError('任务名称不能为空，且不能超过 200 个字符。')
    if not any(t['id'] == tid for t in task_list()):
        raise ValueError('任务不存在。')
    path = TASK_ROOT / tid / 'task.json'
    obj = json.loads(path.read_text(encoding='utf-8'))
    obj['name'] = name
    obj['updated_at'] = now()
    atomic_json(path, obj)
    return obj

def scan_recordings(tid):
    base = TASK_ROOT / tid / 'recordings'
    if not base.exists():
        return []
    rows = []
    for folder in sorted((x for x in base.iterdir() if x.is_dir()), reverse=True):
        path = folder / 'capture_manifest.json'
        try:
            m = json.loads(path.read_text(encoding='utf-8'))
            eps = m.get('episodes', [])
            status = '中断，已保存前缀' if any(e.get('status') == 'interrupted_by_error' for e in eps) else ('已保存' if eps else '未完成')
            frames = sum(int(e.get('frames', 0)) for e in eps)
            rows.append((folder, m, status, frames))
        except (OSError, ValueError, TypeError):
            rows.append((folder, None, '未完成 / 保存失败', 0))
    return rows

def episode_numbers(tid, rows):
    """Persist stable display IDs; recordings are independent LeRobot datasets."""
    path = TASK_ROOT / tid / 'episode_numbers.json'
    if path.exists():
        data = json.loads(path.read_text(encoding='utf-8'))
        numbers = data['recordings']
        next_id = data['next_id']
        if (not isinstance(numbers, dict) or not isinstance(next_id, int)
                or next_id < 0 or any(not isinstance(n, int) or n < 0 for n in numbers.values())
                or len(set(numbers.values())) != len(numbers)
                or (numbers and next_id <= max(numbers.values()))):
            raise ValueError('Episode 编号文件无效，不能安全分配编号。')
    else:
        numbers, next_id = {}, 0
    changed = not path.exists()
    for folder, _, _, _ in sorted(rows, key=lambda item: item[0].name):
        if folder.name not in numbers:
            numbers[folder.name] = next_id
            next_id += 1
            changed = True
    if changed:
        atomic_json(path, {'recordings': numbers, 'next_id': next_id})
    return numbers

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ModuleNotFoundError:
    # Ubuntu 24.04 provides PyQt5 for Python 3.12; keep LeRobot's packages first.
    sys.path.append('/usr/lib/python3/dist-packages')
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    finally:
        # Do not let unrelated system SciPy/NumPy shadow the conda environment.
        sys.path.remove('/usr/lib/python3/dist-packages')

Qt = QtCore.Qt

class App(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.task_id = None
        self.process = None
        self.phase = 'idle'
        self.closing = False
        self.events = queue.Queue()
        self.record_rows = []
        self.replay_windows = set()
        self.ssh_session = tempfile.TemporaryDirectory(prefix='g2ssh-')
        self.connected_host = None
        self.connection_busy = False
        self.reset_busy = False
        self.check_busy = False
        self.vr_process = None
        self.vr_host = None
        self.vr_buffer = b''
        self.vr_last_status = 0.0
        self.vr_started_at = 0.0
        self.vr_keys_ready = False
        self.stop_requested = False
        self.saved_episode = False
        self.setWindowTitle('机器人工作台')
        self.setWindowIcon(QtGui.QIcon(str(PROJECT_ROOT/'assets/g2-model-icon.png')))
        self.resize(1600, 1000)
        self.setMinimumSize(1050, 720)
        self.build()
        self.refresh_tasks()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(100)
        self.connection_timer = QtCore.QTimer(self)
        self.connection_timer.timeout.connect(self.check_connection_async)
        self.connection_timer.start(5000)
        self.vr_timer=QtCore.QTimer(self); self.vr_timer.timeout.connect(self.check_vr_monitor); self.vr_timer.start(2000)

    def button(self, text, handler, name=''):
        button = QtWidgets.QPushButton(text)
        if name: button.setObjectName(name)
        button.clicked.connect(handler)
        return button

    def build(self):
        self.setStyleSheet('''
            QMainWindow, QWidget#contentPanel {background:#f0f6fc;}
            QWidget {color:#173653; font-size:20px;}
            QLabel#title {font-size:32px; font-weight:700; color:#123c67;}
            QLabel#subtitle {color:#476987; font-size:17px;}
            QLabel#status {color:#155b92; font-weight:600;}
            QGroupBox {background:#ffffff; border:1px solid #d4e3ef; border-radius:12px; margin-top:16px; padding:18px 16px 14px; font-weight:600;}
            QGroupBox::title {subcontrol-origin:margin; left:14px; padding:0 7px; color:#245c87;}
            QPushButton {background:#ffffff; border:1px solid #a9cce8; border-radius:7px; padding:9px 15px; min-height:26px;}
            QPushButton:hover {background:#e2f2ff; border-color:#398ac5;}
            QPushButton:disabled {color:#788b9c; background:#edf3f8; border-color:#d7e5f0;}
            QPushButton#primary {background:#287fbd; color:white; border:1px solid #287fbd; font-weight:700;}
            QPushButton#primary:hover {background:#1b69a2;}
            QPushButton#stop {color:#a33c48; border-color:#d9a8b1; font-weight:600;}
            QPushButton#stop:disabled {color:#788b9c; border-color:#d7e5f0;}
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {background:#ffffff; border:1px solid #a9cce8; border-radius:6px; padding:6px; min-height:25px;}
            QListWidget, QTableWidget, QPlainTextEdit {background:#ffffff; border:1px solid #c2dced; border-radius:6px; selection-background-color:#cceaff; selection-color:#123c67;}
            QListWidget::item {padding:12px 10px; margin:3px; border-radius:7px;}
            QListWidget::item:selected {background:#dceeff; color:#135c96;}
            QListWidget::item:hover {background:#edf6ff;}
            QTableWidget {gridline-color:#edf2f7; alternate-background-color:#f6faff;}
            QPlainTextEdit {padding:8px; font-size:18px;}
            QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {border:1px solid #2686ca;}
            QScrollBar:vertical {background:#eef4fa; width:10px; margin:0;}
            QScrollBar::handle:vertical {background:#b4cce0; border-radius:5px; min-height:30px;}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {height:0;}
            QHeaderView::section {background:#dcefff; color:#244f72; border:0; border-bottom:1px solid #b8d6ee; padding:8px;}
            QCheckBox {spacing:7px;}
            QCheckBox::indicator {width:20px; height:20px;}
            QScrollArea {border:0; background:transparent;}
            QSplitter::handle {background:#d1e8fa; width:5px;}
        ''')
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central); outer.setContentsMargins(22, 18, 22, 18); outer.setSpacing(12)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('AGIBOT G2  ·  机器人工作台'); title.setObjectName('title'); header.addWidget(title)
        header.addStretch()
        self.status = QtWidgets.QLabel('就绪 · 请选择或创建任务'); self.status.setObjectName('status'); self.status.setWordWrap(True); header.addWidget(self.status)
        outer.addLayout(header)
        self.pages=QtWidgets.QTabWidget();outer.addWidget(self.pages,1)
        capture_page=QtWidgets.QWidget();capture_layout=QtWidgets.QVBoxLayout(capture_page);capture_layout.setContentsMargins(0,0,0,0)
        self.pages.addTab(capture_page,'数据采集')
        splitter = QtWidgets.QSplitter(Qt.Horizontal); capture_layout.addWidget(splitter, 1)
        left = QtWidgets.QWidget(); left_layout = QtWidgets.QVBoxLayout(left); left_layout.setContentsMargins(0,0,12,0)
        task_header=QtWidgets.QHBoxLayout(); task_header.addWidget(QtWidgets.QLabel('任务列表')); task_header.addStretch()
        self.create_btn=self.button('＋ 创建任务', self.create); task_header.addWidget(self.create_btn); left_layout.addLayout(task_header)
        self.tasks=QtWidgets.QListWidget(); self.tasks.currentItemChanged.connect(self.select_task); left_layout.addWidget(self.tasks,1)
        self.rename_btn=self.button('修改任务名称', self.rename); left_layout.addWidget(self.rename_btn)
        self.delete_task_btn=self.button('删除任务',self.delete_task,'stop'); left_layout.addWidget(self.delete_task_btn)
        left.setMinimumWidth(280); splitter.addWidget(left)
        right=QtWidgets.QWidget(); right.setObjectName('contentPanel'); right_layout=QtWidgets.QVBoxLayout(right); right_layout.setContentsMargins(0,0,0,0); right_layout.setSpacing(10)
        row=QtWidgets.QHBoxLayout(); self.current=QtWidgets.QLabel('尚未选择任务'); self.current.setStyleSheet('font-size:24px;font-weight:700;color:#173e61;')
        self.current.setWordWrap(True); row.addWidget(self.current,1)
        self.stop_btn=self.button('■  结束采集',self.stop,'stop'); self.stop_btn.setEnabled(False); row.addWidget(self.stop_btn)
        self.start_btn=self.button('▶  开始采集',self.start,'primary'); row.addWidget(self.start_btn); capture_layout.insertLayout(0, row)
        settings=QtWidgets.QGroupBox('采集设置'); grid=QtWidgets.QGridLayout(settings); grid.setHorizontalSpacing(12); grid.setVerticalSpacing(9)
        self.ssh=QtWidgets.QLineEdit('agi@192.168.31.11'); self.ssh.setMinimumWidth(160)
        self.ssh.textEdited.connect(self.address_changed)
        ssh_widget=QtWidgets.QWidget(); ssh_layout=QtWidgets.QHBoxLayout(ssh_widget)
        ssh_layout.setContentsMargins(0,0,0,0); ssh_layout.setSpacing(8)
        ssh_layout.addWidget(self.ssh,1)
        self.connect_btn=self.button('连接',self.connect_robot); ssh_layout.addWidget(self.connect_btn)
        self.connection_status=QtWidgets.QLabel('未连接'); self.connection_status.setMinimumWidth(95)
        ssh_layout.addWidget(self.connection_status)
        self.reset_btn=self.button('复位',self.reset_robot);self.reset_btn.setEnabled(False);ssh_layout.addWidget(self.reset_btn)
        self.fps=QtWidgets.QSpinBox(); self.fps.setRange(1,120); self.fps.setValue(30); self.fps.setSuffix(' fps')
        self.width=QtWidgets.QSpinBox(); self.width.setRange(0,7680); self.width.setSingleStep(2); self.width.setValue(640); self.width.setSuffix(' px')
        self.duration=QtWidgets.QDoubleSpinBox(); self.duration.setRange(0,86400); self.duration.setDecimals(1); self.duration.setSuffix(' 秒'); self.duration.setSpecialValueText('手动结束')
        self.simulate=QtWidgets.QCheckBox('模拟测试（不连接机器人）')
        connection_row=QtWidgets.QHBoxLayout();connection_row.addWidget(QtWidgets.QLabel('机器人 SSH'));connection_row.addWidget(ssh_widget,1);outer.insertLayout(1,connection_row)
        self.vr_status=QtWidgets.QLabel('PICO：等待连接机器人');self.vr_status.setWordWrap(True)
        vrrow=QtWidgets.QWidget();vrlayout=QtWidgets.QHBoxLayout(vrrow);vrlayout.setContentsMargins(0,0,0,0)
        vrlayout.addWidget(self.vr_status,1)
        grid.addWidget(QtWidgets.QLabel('VR 状态'),1,0);grid.addWidget(vrrow,1,1,1,7)
        for col,(label,widget) in enumerate([('帧率',self.fps),('最大宽度',self.width),('采集时长',self.duration)]):
            grid.addWidget(QtWidgets.QLabel(label),2,col*2); grid.addWidget(widget,2,col*2+1)
        grid.addWidget(self.simulate,7,0,1,8)
        core=QtWidgets.QLabel('必采数据：关节状态 · 目标 action · 左右夹爪 · 时间戳'); core.setWordWrap(True); core.setObjectName('subtitle'); grid.addWidget(core,3,0,1,8)
        self.cam_vars={}
        grid.addWidget(QtWidgets.QLabel('采集相机：'),4,0)
        for i,(key,label) in enumerate(CAMERAS):
            check=QtWidgets.QCheckBox(label); check.setChecked(i<3); self.cam_vars[key]=check
            grid.addWidget(check,5+i//4,(i%4)*2,1,2)
        note=QtWidgets.QLabel('至少选择一路相机；底盘、深度和 IMU 当前尚未接入。'); note.setObjectName('subtitle'); note.setWordWrap(True); grid.addWidget(note,8,0,1,8)
        from camera_preview import CameraPreview
        self.camera_preview=CameraPreview(CAMERAS,self)
        grid.addWidget(self.camera_preview,9,0,1,8)
        right_layout.addWidget(settings)
        data=QtWidgets.QGroupBox('任务数据'); data_layout=QtWidgets.QVBoxLayout(data)
        self.records=QtWidgets.QTableWidget(0,5); self.records.setHorizontalHeaderLabels(['采集记录','帧数','FPS','相机数','状态'])
        self.records.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows); self.records.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.records.setAlternatingRowColors(True); self.records.setShowGrid(False)
        self.records.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers); self.records.verticalHeader().setVisible(False)
        self.records.horizontalHeader().setSectionResizeMode(0,QtWidgets.QHeaderView.Stretch)
        for i in range(1,5): self.records.horizontalHeader().setSectionResizeMode(i,QtWidgets.QHeaderView.ResizeToContents)
        self.records.verticalHeader().setDefaultSectionSize(42); self.records.setMinimumHeight(155); self.records.itemSelectionChanged.connect(self.view_record); data_layout.addWidget(self.records,1)
        actions=QtWidgets.QHBoxLayout(); actions.addWidget(self.button('查看任务数据',self.view_record)); actions.addWidget(self.button('打开数据文件夹',self.open_record))
        actions.addWidget(self.button('可视化回放',self.open_replay)); actions.addWidget(self.button('真机回放',self.open_robot_compare))
        self.delete_record_btn=self.button('删除记录',self.delete_record,'stop'); actions.addWidget(self.delete_record_btn)
        actions.addStretch(); actions.addWidget(self.button('刷新',self.refresh_records)); data_layout.addLayout(actions)
        self.info=QtWidgets.QPlainTextEdit(); self.info.setReadOnly(True); self.info.setMinimumHeight(145); data_layout.addWidget(self.info,1); right_layout.addWidget(data,1)
        logs=QtWidgets.QGroupBox('运行日志'); logs_layout=QtWidgets.QVBoxLayout(logs); self.log=QtWidgets.QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000); self.log.setMinimumHeight(65); self.log.setMaximumHeight(95); logs_layout.addWidget(self.log); right_layout.addWidget(logs)
        right.setMinimumWidth(920)
        scroll=QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(right)
        splitter.addWidget(scroll); splitter.setSizes([300,1270])
        from robot_operation import OperationPage
        self.operation=OperationPage(self);self.pages.addTab(self.operation,'机器人操作')
        self.pages.currentChanged.connect(lambda index:self.operation.disarm())

    def append(self, value):
        self.log.appendPlainText(value)

    def ssh_options(self):
        return ['-o','ControlPath='+str(Path(self.ssh_session.name)/'%C'),
                '-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3']

    def master_is_alive(self,host):
        try:
            result=subprocess.run(['ssh',*self.ssh_options(),'-O','check',host],
                    stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=3)
            return result.returncode==0
        except (OSError,subprocess.TimeoutExpired): return False

    def address_changed(self):
        if self.connected_host and self.ssh.text().strip()!=self.connected_host:
            self.connection_status.setText('地址已更改'); self.stop_vr_monitor(); self.vr_status.setText('PICO：请先连接当前机器人')
        elif self.connected_host:
            self.connection_status.setText('已连接')
        else:
            self.connection_status.setText('未连接')

    def connect_robot(self):
        if self.reset_busy:return
        if self.process or self.connection_busy: return
        host=self.ssh.text().strip()
        if not host or host.startswith('-') or any(c.isspace() for c in host):
            QtWidgets.QMessageBox.warning(self,'连接机器人','请填写 SSH 地址，例如 agi@192.168.31.11。'); return
        self.connection_busy=True; self.connect_btn.setEnabled(False); self.ssh.setEnabled(False)
        self.connection_status.setText('连接中…'); self.append('正在连接 '+host)
        def worker():
            env=os.environ.copy()
            env.update({'SSH_ASKPASS':str(HERE/'ssh_askpass.py'),'SSH_ASKPASS_REQUIRE':'force'})
            try:
                if self.master_is_alive(host):
                    self.events.put(('connected',(host,True,''))); return
                cmd=['ssh',*self.ssh_options(),'-o','ControlMaster=auto','-o','ControlPersist=yes','-MNf',host]
                result=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,
                                      stderr=subprocess.PIPE,text=True,errors='replace',env=env,timeout=180)
                ok=result.returncode==0 and self.master_is_alive(host)
                self.events.put(('connected',(host,ok,result.stderr.strip())))
            except (OSError,subprocess.TimeoutExpired) as e:
                self.events.put(('connected',(host,False,str(e))))
        threading.Thread(target=worker,daemon=True).start()

    def check_connection_async(self):
        if not self.connected_host or self.connection_busy or self.check_busy: return
        host=self.connected_host; self.check_busy=True
        def worker(): self.events.put(('connection_check',(host,self.master_is_alive(host))))
        threading.Thread(target=worker,daemon=True).start()

    def start_vr_monitor(self):
        host=self.connected_host
        if not host or host!=self.ssh.text().strip() or self.vr_process:return
        try:
            code=(HERE/'vr_buttons_source.py').read_text(encoding='utf-8')
            remote='source /home/agi/app/env.sh >/dev/null 2>&1 && exec python3 -u -c '+shlex.quote(code)
            args=['-T',*self.ssh_options(),'-o','BatchMode=yes','-o','ControlMaster=no',host,'bash -lc '+shlex.quote(remote)]
            proc=QtCore.QProcess(self);self.vr_process=proc;self.vr_host=host;self.vr_buffer=b''
            self.vr_last_status=0.;self.vr_started_at=time.monotonic();self.vr_keys_ready=False
            proc.readyReadStandardOutput.connect(self.read_vr_output)
            proc.readyReadStandardError.connect(lambda: proc.readAllStandardError())
            proc.finished.connect(lambda *unused: self.vr_monitor_finished(proc))
            proc.start('ssh',args)
            self.vr_status.setText('PICO：正在连接状态与按键监测…')
        except (OSError,ValueError) as exc:
            self.vr_status.setText('PICO：无法启动状态监测');self.append('VR 状态监测失败：'+str(exc))

    def stop_vr_monitor(self):
        proc=self.vr_process;self.vr_process=None;self.vr_host=None;self.vr_keys_ready=False;self.vr_last_status=0.
        if proc:
            proc.terminate()
            if not proc.waitForFinished(1000):proc.kill();proc.waitForFinished(1000)
            proc.deleteLater()

    def vr_monitor_finished(self,proc):
        if self.vr_process is proc:
            self.vr_process=None;self.vr_keys_ready=False;self.vr_last_status=0.
            self.vr_status.setText('PICO：状态监测已断开，正在重试…')
        proc.deleteLater()

    def check_vr_monitor(self):
        if not self.connected_host or self.connected_host!=self.ssh.text().strip():
            if self.vr_process:self.stop_vr_monitor()
            return
        if self.vr_process is None:
            self.start_vr_monitor();return
        if (self.vr_last_status and time.monotonic()-self.vr_last_status>4) or (not self.vr_last_status and time.monotonic()-self.vr_started_at>15):
            self.vr_keys_ready=False;self.vr_status.setText('PICO：状态更新中断，正在重试…');self.stop_vr_monitor()

    def read_vr_output(self):
        if not self.vr_process:return
        self.vr_buffer+=bytes(self.vr_process.readAllStandardOutput())
        if len(self.vr_buffer)>65536:
            self.stop_vr_monitor();self.vr_status.setText('PICO：状态数据异常');return
        while b'\n' in self.vr_buffer:
            line,self.vr_buffer=self.vr_buffer.split(b'\n',1)
            try:data=json.loads(line)
            except (ValueError,UnicodeDecodeError):continue
            if self.vr_host!=self.connected_host or self.vr_host!=self.ssh.text().strip():continue
            if data.get('kind')=='status':
                self.vr_last_status=time.monotonic()
                connected=data.get('vr_connected');stream=data.get('key_stream') is True
                self.vr_keys_ready=connected is True and stream
                usb='USB 已连接' if data.get('usb_connected') else 'USB 未检测到'
                link='Genie Link 已连接' if connected is True else ('Genie Link 未连接' if connected is False else 'Genie Link 状态未知')
                keys='X 开始 / 停止采集' if self.vr_keys_ready else '按键不可用'
                self.vr_status.setText('PICO：'+usb+' · '+link+' · '+keys)
            elif data.get('kind')=='key' and self.vr_keys_ready and time.monotonic()-self.vr_last_status<3:
                key=data.get('key')
                if key=='X':
                    if self.phase=='stopping':
                        self.append('VR X：正在保存，请稍候')
                    elif self.process:
                        self.append('VR X：请求停止采集');self.stop()
                    else:
                        self.append('VR X：请求开始采集');self.start()
            elif data.get('kind')=='error':
                self.vr_keys_ready=False;self.vr_status.setText('PICO：按键监听失败');self.append('VR 状态监听：'+str(data.get('message','')))

    def reset_robot(self):
        if hasattr(self,'operation'):
            self.operation.shutdown()
            if self.operation.stopping:
                self.status.setText('正在停止手动操作通道，请稍后再次点击复位');return
        host=self.ssh.text().strip()
        if self.reset_busy:return
        if self.connected_host!=host or not self.master_is_alive(host):
            QtWidgets.QMessageBox.information(self,'复位','请先连接机器人 SSH。');return
        if self.process or any(getattr(w,'motion',None) is not None for w in self.replay_windows):
            QtWidgets.QMessageBox.information(self,'复位','请先停止采集和真机回放。');return
        if QtWidgets.QMessageBox.question(self,'机器人姿态复位',
            '将通过 GDK 规划复位到 OmniPicker 预设姿态（pose 11），移动头部、躯干和双臂，速度 0.2 rad/s；无需 VR。请停止其他运动控制，确认运动范围内无障碍。是否复位？',
            QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)!=QtWidgets.QMessageBox.Yes:return
        if self.process or self.connected_host!=host or self.ssh.text().strip()!=host or any(getattr(w,'motion',None) is not None for w in self.replay_windows):
            QtWidgets.QMessageBox.information(self,'复位','连接或任务状态已变化，请停止其他任务后重试。');return
        self.reset_busy=True;self.reset_btn.setEnabled(False);self.status.setText('正在执行独立 GDK 复位并核对关节位置…')
        def run():
            try:
                script=(HERE/'robot_reset.py').read_text()
                remote='source /home/agi/app/env.sh >/dev/null 2>&1 && exec python3 -u -c '+shlex.quote(script)
                result=subprocess.run(['ssh','-T',*self.ssh_options(),'-o','BatchMode=yes','-o','ControlMaster=no',host,
                    'bash -lc '+shlex.quote(remote)],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                    text=True,timeout=65)
                messages=[line[len('RESET_RESULT '):] for line in result.stdout.splitlines() if line.startswith('RESET_RESULT ')]
                if not messages:raise RuntimeError('未收到复位回执；动作结果未知，请观察机器人，勿重复点击。'+result.stderr[-300:])
                answer=json.loads(messages[-1]);self.events.put(('reset_done',answer))
            except subprocess.TimeoutExpired:
                self.events.put(('reset_done',{'ok':False,'message':'复位请求超时，动作结果未知；请观察机器人，不会自动重试。'}))
            except Exception as exc:self.events.put(('reset_done',{'ok':False,'message':str(exc)}))
        threading.Thread(target=run,daemon=True).start()

    def refresh_tasks(self, select=None):
        tasks=task_list(); self.tasks.blockSignals(True); self.tasks.clear()
        for task in tasks:
            item=QtWidgets.QListWidgetItem(task['name']); item.setData(Qt.UserRole,task['id']); self.tasks.addItem(item)
            if task['id']==select: self.tasks.setCurrentItem(item)
        self.tasks.blockSignals(False)
        if select and any(t['id']==select for t in tasks): self.task_id=select
        elif not tasks: self.task_id=None
        self.current.setText(next((t['name'] for t in tasks if t['id']==self.task_id),'尚未选择任务'))
        self.refresh_records()

    def selected_task(self):
        return next((t for t in task_list() if t['id']==self.task_id),None)

    def create(self):
        if self.process: return
        name,ok=QtWidgets.QInputDialog.getText(self,'创建任务','请输入任务名称 / 采集任务描述：')
        if not ok: return
        try:
            task=create_task(name); self.refresh_tasks(task['id']); self.status.setText('任务已创建')
        except (OSError,ValueError) as e: QtWidgets.QMessageBox.critical(self,'创建失败',str(e))

    def rename(self):
        if self.process: return
        task=self.selected_task()
        if not task: return
        name,ok=QtWidgets.QInputDialog.getText(self,'修改任务名称','新名称（历史数据的任务描述保持不变）：',text=task['name'])
        if not ok: return
        try: rename_task(task['id'],name); self.refresh_tasks(task['id'])
        except (OSError,ValueError) as e: QtWidgets.QMessageBox.critical(self,'修改失败',str(e))

    def delete_task(self):
        if self.process: return
        task=self.selected_task()
        if not task:
            QtWidgets.QMessageBox.information(self,'删除任务','请先选择任务。'); return
        reply=QtWidgets.QMessageBox.question(self,'删除任务',
            f'确定永久删除任务“{task["name"]}”及其所有采集记录？此操作无法撤销。',
            QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)
        if reply != QtWidgets.QMessageBox.Yes: return
        try:
            folder=TASK_ROOT/task['id']
            if folder.is_symlink() or folder.parent != TASK_ROOT: raise ValueError('任务目录不安全')
            shutil.rmtree(folder)
            self.task_id=None; self.refresh_tasks(); self.status.setText('任务及其数据已删除')
        except (OSError,ValueError) as e: QtWidgets.QMessageBox.critical(self,'删除失败',str(e))

    def delete_record(self):
        if self.process: return
        item=self.selected_record()
        if not item:
            QtWidgets.QMessageBox.information(self,'删除记录','请先选择一条采集记录。'); return
        folder=item[0]
        label=self.records.item(self.records.currentRow(),0).text()
        reply=QtWidgets.QMessageBox.question(self,'删除采集记录',
            f'确定永久删除 {label} 的数据？此操作无法撤销。',
            QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)
        if reply != QtWidgets.QMessageBox.Yes: return
        try:
            base=TASK_ROOT/self.task_id/'recordings'
            if folder.is_symlink() or folder.parent != base or base.is_symlink(): raise ValueError('记录目录不安全')
            shutil.rmtree(folder); self.refresh_records(); self.status.setText('采集记录已删除')
        except (OSError,ValueError) as e: QtWidgets.QMessageBox.critical(self,'删除失败',str(e))

    def select_task(self,current,previous):
        if self.process:
            self.tasks.blockSignals(True); self.tasks.setCurrentItem(previous); self.tasks.blockSignals(False); return
        self.task_id=current.data(Qt.UserRole) if current else None
        task=self.selected_task(); self.current.setText(task['name'] if task else '尚未选择任务'); self.refresh_records()

    def refresh_records(self):
        self.records.setRowCount(0); self.record_rows=[]; self.info.clear()
        if not self.task_id: return
        self.record_rows=scan_recordings(self.task_id)
        try:
            numbers=episode_numbers(self.task_id,self.record_rows)
        except (OSError,ValueError,KeyError,TypeError) as e:
            self.info.setPlainText('无法读取 Episode 编号：'+str(e)); self.record_rows=[]; return
        self.record_rows.sort(key=lambda item:numbers[item[0].name])
        for row,(folder,m,status,frames) in enumerate(self.record_rows):
            self.records.insertRow(row); schema=(m or {}).get('schema',{})
            label='Episode '+str(row)+(' [模拟]' if (m or {}).get('simulated') else '')
            for col,val in enumerate([label,frames,(m or {}).get('fps','-'),len(schema.get('cameras',[])),status]):
                self.records.setItem(row,col,QtWidgets.QTableWidgetItem(str(val)))
        if self.record_rows: self.records.selectRow(0); self.view_record()
        else: self.info.setPlainText('此任务还没有采集记录。')

    def selected_record(self):
        row=self.records.currentRow()
        return self.record_rows[row] if 0 <= row < len(self.record_rows) else None

    def view_record(self):
        item=self.selected_record(); self.info.clear()
        if not item: self.info.setPlainText('请选择一条采集记录。'); return
        folder,manifest,status,frames=item
        if not manifest: self.info.setPlainText(f'状态：{status}\n目录：{folder}\n未找到完整采集清单。'); return
        schema=manifest.get('schema',{}); cameras=schema.get('cameras',[])
        lines=[f'状态：{status}    类型：{"模拟测试" if manifest.get("simulated") else "机器人实采"}',
               f'帧数：{frames}    帧率：{manifest.get("fps", "-")} FPS    相机：{", ".join(cameras)}',
               f'关节数：{len(schema.get("joint_names",[]))}    目录：{folder}']
        try:
            import pyarrow.parquet as pq
            files=sorted((folder/'data').rglob('*.parquet'))
            if files:
                columns=['observation.state','action']+(['observation.gripper'] if schema.get('gripper_names') else [])
                row=pq.read_table(files[0],columns=columns).slice(0,1).to_pylist()[0]
                for key,label in [('observation.state','首帧状态（rad）'),('action','首帧 action（rad）')]:
                    lines.append(label+'：'+', '.join(f'{x:.4f}' for x in row[key]))
                if 'observation.gripper' in row: lines.append('首帧夹爪（左、右，rad）：'+', '.join(f'{x:.4f}' for x in row['observation.gripper']))
            tasks=folder/'meta/tasks.parquet'
            if tasks.exists(): lines.append('数据集内任务描述：'+str(pq.read_table(tasks).to_pandas().index.tolist()))
        except Exception as e: lines.append('读取数据预览失败：'+str(e))
        for episode in manifest.get('episodes',[]):
            if episode.get('error'): lines.append('采集中断原因：'+episode['error'])
        self.info.setPlainText('\n'.join(lines))

    def open_record(self):
        item=self.selected_record()
        if not item: QtWidgets.QMessageBox.information(self,'查看数据','请先选择一条采集记录。'); return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(item[0])))

    def open_replay(self):
        item=self.selected_record()
        if not item or not item[1]:
            QtWidgets.QMessageBox.information(self,'可视化回放','请选择一条已保存的采集记录。'); return
        try:
            from replay import ReplayDialog
            window=ReplayDialog(item[0],self)
            self.replay_windows.add(window)
            window.destroyed.connect(lambda: self.replay_windows.discard(window))
            window.show()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self,'可视化回放失败',str(exc))

    def open_robot_compare(self):
        self.operation.shutdown()
        if self.operation.stopping:
            self.status.setText('正在停止手动操作通道，请稍后再次打开真机回放');return
        if self.reset_busy:return
        item=self.selected_record()
        if not item or not item[1]:
            QtWidgets.QMessageBox.information(self,'真机画面对照','请先选择一条已保存的采集记录。'); return
        host=self.ssh.text().strip()
        if self.connected_host != host:
            QtWidgets.QMessageBox.information(self,'真机画面对照','请先连接机器人 SSH。'); return
        try:
            from robot_compare import CompareDialog
            window=CompareDialog(item[0],host,self.ssh_session.name,self)
            self.replay_windows.add(window)
            window.destroyed.connect(lambda: self.replay_windows.discard(window))
            window.showMaximized()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self,'真机画面对照失败',str(exc))

    def start(self):
        if self.reset_busy:return
        if self.process: return
        task=self.selected_task()
        if not task: QtWidgets.QMessageBox.information(self,'开始采集','请先创建并选择任务。'); return
        cameras=[k for k,_ in CAMERAS if self.cam_vars[k].isChecked()]
        if not cameras: QtWidgets.QMessageBox.warning(self,'采集设置','请至少选择一路相机。'); return
        fps=self.fps.value(); width=self.width.value(); duration=self.duration.value(); ssh=self.ssh.text().strip()
        if width%2: QtWidgets.QMessageBox.warning(self,'采集设置','最大宽度须为 0 或偶数。'); return
        if not self.simulate.isChecked() and (not ssh or ssh.startswith('-') or any(c.isspace() for c in ssh)):
            QtWidgets.QMessageBox.warning(self,'采集设置','请填写 SSH 地址，例如 agi@192.168.31.11。'); return
        if not self.simulate.isChecked() and (self.connection_busy or self.connected_host!=ssh or not self.master_is_alive(ssh)):
            self.connected_host=None; self.connection_status.setText('未连接')
            QtWidgets.QMessageBox.warning(self,'开始采集','请先点击机器人 SSH 旁的“连接”，成功后再开始采集。'); return
        base=TASK_ROOT/task['id']/'recordings'; base.mkdir(exist_ok=True)
        out=base/(dt.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        cmd=[sys.executable,'-u',str(HERE/'collect.py'),str(out),'--task',task['name'],'--fps',str(fps),'--max-width',str(width),
             '--duration',str(duration),'--start-delay','3','--yes','--ssh',ssh,'--cameras',*cameras]
        if self.simulate.isChecked(): cmd.append('--simulate')
        else: cmd.extend(['--ssh-control-dir',self.ssh_session.name,'--require-ssh-control'])
        env=os.environ.copy(); env.update({'SSH_ASKPASS':str(HERE/'ssh_askpass.py'),'SSH_ASKPASS_REQUIRE':'force','PYTHONUNBUFFERED':'1'})
        try:
            self.process=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                text=True,encoding='utf-8',errors='replace',bufsize=1,env=env,start_new_session=True)
        except OSError as e: QtWidgets.QMessageBox.critical(self,'启动失败',str(e)); return
        self.phase='starting'; self.stop_requested=False; self.saved_episode=False
        self.output_path=out; self.status.setText('正在初始化，准备 3 秒倒计时'+(' · 模拟测试' if self.simulate.isChecked() else ''))
        self.append('\n开始：'+str(out)); self.start_btn.setEnabled(False); self.stop_btn.setEnabled(True)
        self.create_btn.setEnabled(False); self.rename_btn.setEnabled(False); self.delete_task_btn.setEnabled(False); self.delete_record_btn.setEnabled(False)
        self.connect_btn.setEnabled(False); self.ssh.setEnabled(False)
        threading.Thread(target=self.read_process,args=(self.process,),daemon=True).start()

    def read_process(self,proc):
        for line in proc.stdout: self.events.put(('line',line.rstrip()))
        proc.stdout.close(); self.events.put(('done',proc.wait()))

    def stop(self):
        if not self.process or self.phase=='stopping': return
        if self.process.poll() is None:
            self.stop_requested=True; self.phase='stopping'; self.status.setText('正在停止采集，正在保存…'); self.append('正在停止采集，正在保存…'); self.stop_btn.setEnabled(False)
            try: os.kill(self.process.pid,signal.SIGINT)
            except ProcessLookupError: pass

    def poll(self):
        preview_host=self.connected_host if self.connected_host==self.ssh.text().strip() and self.pages.currentIndex()==0 else None
        self.camera_preview.configure(preview_host,self.ssh_session.name,[n for n,_ in CAMERAS if self.cam_vars[n].isChecked()],self.simulate.isChecked())
        for check in self.cam_vars.values():check.setEnabled(self.process is None)
        self.reset_btn.setEnabled(bool(self.connected_host and self.connected_host==self.ssh.text().strip() and not self.connection_busy and not self.reset_busy and not self.process and not any(getattr(w,'motion',None) is not None for w in self.replay_windows)))
        try:
            while True:
                typ,value=self.events.get_nowait()
                if typ=='reset_done':
                    self.reset_busy=False
                    message=value['message'];self.status.setText(message);self.append('复位：'+message)
                    if not value['ok']:QtWidgets.QMessageBox.warning(self,'复位请求',message)
                elif typ=='connected':
                    host,ok,error=value
                    self.connection_busy=False; self.connect_btn.setEnabled(not bool(self.process)); self.ssh.setEnabled(not bool(self.process))
                    if ok:
                        self.connected_host=host; self.connection_status.setText('已连接'); self.append('SSH 已连接：'+host); self.vr_status.setText('PICO：正在检查…'); self.start_vr_monitor()
                    else:
                        self.connected_host=None; self.connection_status.setText('连接失败'); self.vr_status.setText('PICO：机器人未连接')
                        self.append('SSH 连接失败：'+(error or '无法建立 SSH 连接'))
                        self.status.setText('连接失败，请检查网络和账号后重试')
                    if self.closing: self.close(); return
                elif typ=='connection_check':
                    host,alive=value; self.check_busy=False
                    if host==self.connected_host and not alive:
                        self.connected_host=None; self.connection_status.setText('连接已断开'); self.stop_vr_monitor(); self.vr_status.setText('PICO：机器人连接已断开'); self.append('SSH 连接已断开，请重新连接。')
                elif typ=='line':
                    self.append(value)
                    if value.startswith('COUNTDOWN ') and self.phase=='starting':
                        self.status.setText('准备开始采集：'+value.split()[-1]+' 秒')
                    elif value.startswith('RECORDING') and self.phase=='starting':
                        self.phase='recording'; self.status.setText('已开始采集'+(' · 模拟测试' if self.simulate.isChecked() else ''))
                        self.append('已开始采集'); QtWidgets.QApplication.beep()
                    elif value.startswith('episode=') and self.phase=='recording': self.status.setText('采集中 · '+value)
                    elif value.startswith('Saved episode '): self.saved_episode=True
                else:
                    self.process=None; self.phase='idle'; self.start_btn.setEnabled(True); self.stop_btn.setEnabled(False)
                    self.create_btn.setEnabled(True); self.rename_btn.setEnabled(True); self.delete_task_btn.setEnabled(True); self.delete_record_btn.setEnabled(True); self.refresh_records()
                    self.connect_btn.setEnabled(not self.connection_busy); self.ssh.setEnabled(not self.connection_busy)
                    if self.stop_requested and self.saved_episode and value==0:
                        self.status.setText('采集已停止，数据已保存'); self.append('采集已停止，数据已保存'); QtWidgets.QApplication.beep()
                    elif self.stop_requested and not self.saved_episode:
                        self.status.setText('采集已停止，未保存数据'); self.append('采集已停止，未保存数据'); QtWidgets.QApplication.beep()
                    elif self.stop_requested:
                        self.status.setText('采集已停止，保存异常，请查看日志'); self.append('采集已停止，保存异常，请查看日志'); QtWidgets.QApplication.beep()
                    elif value==0:
                        self.status.setText('采集已结束，数据已保存'); self.append('采集已结束，数据已保存'); QtWidgets.QApplication.beep()
                    else:
                        self.status.setText('采集失败或中断，请查看日志')
                    self.append('采集进程退出，状态码：'+str(value))
                    if self.closing: self.close(); return
        except queue.Empty: pass

    def closeEvent(self,event):
        self.operation.shutdown()
        if self.reset_busy:
            self.status.setText('正在等待复位请求结果，请稍候…');event.ignore();return
        if self.connection_busy:
            QtWidgets.QMessageBox.information(self,'连接中','请等待 SSH 连接完成或取消密码输入后再退出。')
            event.ignore(); return
        if self.process:
            reply=QtWidgets.QMessageBox.question(self,'退出','采集仍在进行。结束当前采集并等待保存后退出？',QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No)
            if reply==QtWidgets.QMessageBox.Yes: self.closing=True; self.stop()
            event.ignore()
        else:
            if not self.operation.close_all():
                event.ignore();return
            if not self.camera_preview.shutdown():
                self.status.setText('正在关闭相机预览，请稍候…');event.ignore();return
            self.vr_timer.stop();self.stop_vr_monitor()
            # Close only masters started by this GUI, then remove the private socket directory.
            for socket in Path(self.ssh_session.name).iterdir():
                try: subprocess.run(['ssh','-S',str(socket),'-O','exit','localhost'],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2)
                except (OSError,subprocess.TimeoutExpired): pass
            self.ssh_session.cleanup(); event.accept()

def main():
    app=QtWidgets.QApplication(sys.argv); window=App(); window.show(); return app.exec_()

if __name__=='__main__':
    sys.exit(main())
