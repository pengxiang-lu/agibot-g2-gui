"""Speech and face controls; background SSH jobs reuse the GUI connection."""
import hashlib, json, os, re, shlex, subprocess, tempfile, threading, time, uuid
from pathlib import Path
from PyQt5 import QtCore, QtWidgets
HERE=Path(__file__).resolve().parent
RESULT='G2_MEDIA_RESULT='
BUILTIN_ROOT='/agibot/software/agent/bin/cfg/face_ui'
LABELS={'idle':'默认待机','happy':'开心','wink':'眨眼','shy':'害羞','smile':'微笑','sorry':'抱歉','thinking':'思考','boring':'无聊','warning':'警告','turnoff_screen':'息屏','idle_gt_consultant':'待机顾问','default':'默认','listening':'聆听','speaking':'说话','understanding':'思考','playing':'播放','start':'启动','charged':'充电完成','charging':'充电中','low_power':'低电量','machine_error':'故障','machine_stop':'停止'}

def run(args, *, data=None, timeout=30, env=None):
    p=subprocess.run(args,input=data,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=timeout,env=env)
    if p.returncode:
        for line in p.stdout.decode(errors='replace').splitlines():
            if line.startswith(RESULT):
                try:detail=json.loads(line[len(RESULT):]).get('error')
                except ValueError:detail=None
                if detail:raise RuntimeError(detail)
        raise RuntimeError(p.stderr.decode(errors='replace').strip()[-1200:] or '操作失败')
    return p.stdout.decode(errors='replace')

def parse_result(text, prefix=''):
    for line in reversed(text.splitlines()):
        if prefix and not line.startswith(prefix):continue
        try:return json.loads(line[len(prefix):])
        except ValueError:continue
    raise RuntimeError('未收到有效响应')

def convert_media(source, target):
    source=Path(source)
    if not source.is_file():raise ValueError('所选文件不存在')
    if source.stat().st_size>512*1024*1024:raise ValueError('请选择小于 512 MB 的图片或视频')
    image=source.suffix.lower() in ('.png','.jpg','.jpeg','.bmp','.webp')
    args=['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y']
    if image:args+=['-loop','1']
    args+=['-i',str(source),'-filter_complex',
           'color=c=black:s=2670x1200:r=25[bg];'
           '[0:v:0]scale=2670:1200:force_original_aspect_ratio=increase,'
           'crop=2670:1200,setsar=1[face];'
           '[bg][face]overlay=shortest=1:format=auto,format=yuv420p[out]',
           '-map','[out]','-an','-t','8' if image else '120',
           '-r','25','-c:v','libx264','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(target)]
    run(args,timeout=240)

def convert_audio(source,target):
    source=Path(source)
    if not source.is_file():raise ValueError('所选音频不存在')
    if source.stat().st_size>512*1024*1024:raise ValueError('请选择小于 512 MB 的音频')
    run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-y','-i',str(source),
         '-map','0:a:0','-vn','-ac','1','-ar','16000','-c:a','pcm_s16le',str(target)],timeout=240)

class InteractionPanel(QtWidgets.QGroupBox):
    completed=QtCore.pyqtSignal(object)
    progress=QtCore.pyqtSignal(str)
    connection_checked=QtCore.pyqtSignal(object)
    def __init__(self,owner,operation):
        super().__init__('语音与表情');self.owner=owner;self.operation=operation;self.working=False;self.head_key=None;self.closing=False;self.connecting=False;self.checking=False;self.last_check=0
        layout=QtWidgets.QVBoxLayout(self);self.controls=[]
        connection=QtWidgets.QHBoxLayout();connection.addWidget(QtWidgets.QLabel('aispeech SSH'))
        self.head=QtWidgets.QLineEdit('agi@10.42.0.111');self.head.setToolTip('经机器人主机连接交互板；点击连接验证密码，后续复用会话。');connection.addWidget(self.head,1)
        self.connect_btn=QtWidgets.QPushButton('连接');self.connect_btn.clicked.connect(self.connect_clicked);connection.addWidget(self.connect_btn)
        self.connection_status=QtWidgets.QLabel('未连接');connection.addWidget(self.connection_status);layout.addLayout(connection)
        row=QtWidgets.QHBoxLayout();row.addWidget(QtWidgets.QLabel('音量'))
        self.volume=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.volume.setRange(0,100);self.volume.setValue(20)
        self.volume_value=QtWidgets.QSpinBox();self.volume_value.setRange(0,100);self.volume_value.setSuffix('%');self.volume_value.setValue(20)
        self.volume.valueChanged.connect(self.volume_value.setValue);self.volume_value.valueChanged.connect(self.volume.setValue)
        row.addWidget(self.volume,1);row.addWidget(self.volume_value)
        self.button(row,'设置音量',lambda:self.gdk_action({'kind':'volume','volume':self.volume.value()},'音量设置请求已接受'))
        self.button(row,'读取音量',lambda:self.gdk_action({'kind':'status'},'已读取机器人音量'))
        layout.addLayout(row)
        self.text=QtWidgets.QPlainTextEdit();self.text.setPlaceholderText('输入需要机器人播报的文字（最多 1000 字）');self.text.setMaximumHeight(105);layout.addWidget(self.text)
        row=QtWidgets.QHBoxLayout();row.addStretch();self.button(row,'播报',self.speak);self.button(row,'导入音频并播放…',self.upload_audio);layout.addLayout(row)
        row=QtWidgets.QHBoxLayout();row.addWidget(QtWidgets.QLabel('内置表情'))
        self.expressions=QtWidgets.QComboBox();self.expressions.setMinimumWidth(170);row.addWidget(self.expressions,1)
        self.button(row,'刷新表情',self.refresh_faces);self.button(row,'切换表情',self.play_builtin)
        layout.addLayout(row)
        row=QtWidgets.QHBoxLayout();self.loop=QtWidgets.QCheckBox('循环显示');self.loop.setChecked(True);row.addWidget(self.loop)
        self.button(row,'上传表情…',self.upload);self.button(row,'关闭脸屏',lambda:self.gdk_action({'kind':'display_off'},'关闭脸屏请求已接受'))
        row.addStretch();layout.addLayout(row)
        self.status=QtWidgets.QLabel('请先连接 aispeech SSH。图片转为 8 秒静音视频；视频最多取前 120 秒。');self.status.setWordWrap(True);layout.addWidget(self.status)
        self.completed.connect(self.done);self.progress.connect(self.status.setText);self.connection_checked.connect(self.checked)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.update_enabled);self.timer.start(250);self.update_enabled()
    def button(self,row,label,fn):
        b=QtWidgets.QPushButton(label);b.clicked.connect(fn);row.addWidget(b);self.controls.append(b);return b
    def connected(self):return bool(self.owner.connected_host and self.owner.connected_host==self.owner.ssh.text().strip())
    def update_enabled(self):
        valid=self.connected() and self.head_key==(self.owner.connected_host,self.head.text().strip())
        if self.head_key and not valid:
            self.head_key=None;self.connection_status.setText('未连接');self.expressions.clear()
        for b in self.controls:b.setEnabled(valid and not self.working)
        self.connect_btn.setEnabled(self.connected() and not self.working)
        self.connect_btn.setText('连接中…' if self.connecting else ('重新连接' if valid else '连接'))
        self.head.setEnabled(not self.working);self.expressions.setEnabled(valid and not self.working)
        if valid and not self.working and not self.checking and time.monotonic()-self.last_check>=5:
            self.checking=True;self.last_check=time.monotonic();ctx=self.context();key=self.head_key
            def check():
                try:
                    run(['ssh','-T',*self.head_options(ctx),'-o','BatchMode=yes','-o','ControlMaster=no',ctx[2],'true'],timeout=6)
                    ok=True
                except Exception:ok=False
                self.connection_checked.emit((key,ok))
            threading.Thread(target=check,daemon=True).start()
    def checked(self,result):
        self.checking=False;key,ok=result
        if key==self.head_key and not ok:
            self.head_key=None;self.connection_status.setText('连接已断开');self.status.setText('aispeech SSH 已断开，请点击连接后重试。')
        self.update_enabled()
    def connect_clicked(self):
        def work(ctx):
            opts=self.connect_head(ctx,authenticate=True)
            run(['ssh','-T',*opts,'-o','BatchMode=yes','-o','ControlMaster=no',ctx[2],'true'],timeout=8)
            return {'connected_key':(ctx[0],ctx[2]),'message':'aispeech SSH 已连接，可以使用语音和表情功能。'}
        self.launch(work,connection=True)
    def context(self):
        host=self.owner.connected_host
        if not self.connected():raise RuntimeError('请先连接机器人 SSH')
        return host,list(self.owner.ssh_options()),self.head.text().strip()
    def launch(self,fn,connection=False):
        if self.working:return
        try:ctx=self.context()
        except Exception as e:self.status.setText(str(e));return
        if not connection and self.head_key!=(ctx[0],ctx[2]):
            self.status.setText('请先点击 aispeech SSH 右侧的连接按钮。');return
        self.operation.disarm();self.working=True;self.connecting=connection
        if connection:self.head_key=None;self.connection_status.setText('连接中…')
        self.update_enabled();self.status.setText('正在连接 aispeech SSH…' if connection else '正在处理…')
        def worker():
            try:result=fn(ctx);self.completed.emit((True,result))
            except Exception as e:self.completed.emit((False,str(e)))
        threading.Thread(target=worker,daemon=True).start()
    def gdk(self,ctx,request):
        # Native head-board services; the older main-board GDK interaction path
        # does not reach this firmware's Agent service.
        opts=self.connect_head(ctx)
        code=(HERE/'interaction_source.py').read_text()
        remote=('source /opt/ros/humble/setup.bash >/dev/null 2>&1 && '
                'source /agibot/software/hal_audio/share/aimdk_msgs/local_setup.bash >/dev/null 2>&1 && '
                'export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp '
                'FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/app/conf/dds/ros_fastdds.xml; '
                'exec python3 -u -c '+shlex.quote(code))
        result=parse_result(run(['ssh','-T',*opts,'-o','BatchMode=yes','-o','ControlMaster=no',ctx[2],
            'sudo -n bash -lc '+shlex.quote(remote)],data=(json.dumps(request)+'\n').encode(),timeout=55),RESULT)
        if not result.get('ok'):raise RuntimeError(result.get('error','交互请求失败'))
        return result
    def gdk_action(self,request,message):
        self.launch(lambda ctx:{**self.gdk(ctx,request),'message':message})
    def speak(self):
        text=self.text.toPlainText().strip()
        if not 1<=len(text)<=1000:self.status.setText('请输入 1–1000 个字符');return
        self.gdk_action({'kind':'speak','text':text,'volume':self.volume.value()},'播报请求已接受')
    def head_options(self,ctx):
        host,opts,head=ctx
        if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*@10\.42\.0\.111',head):raise ValueError('脸屏地址格式：用户名@10.42.0.111')
        proxy=shlex.join(['ssh','-T',*[v.replace('%','%%') for v in opts],'-o','BatchMode=yes','-o','ControlMaster=no','-W','%h:%p',host])
        head_opts=list(opts)
        for index,value in enumerate(head_opts):
            if value.startswith('ControlPath='):
                directory=Path(value.split('=',1)[1]).parent
                head_opts[index]='ControlPath='+str(directory/('head_'+hashlib.sha256((host+'|'+head).encode()).hexdigest()[:20]))
        return [*head_opts,'-o','ProxyCommand='+proxy]
    def connect_head(self,ctx,authenticate=False):
        opts=self.head_options(ctx);head=ctx[2]
        check=['ssh',*opts,'-O','check',head]
        try:run(check,timeout=5);return opts
        except (RuntimeError,subprocess.TimeoutExpired):pass
        if not authenticate:raise ConnectionError('aispeech SSH 会话已失效，请点击连接按钮重新连接。')
        self.progress.emit('正在连接 aispeech SSH，请完成密码验证…')
        env=os.environ.copy();env.update(SSH_ASKPASS=str(HERE/'ssh_askpass.py'),SSH_ASKPASS_REQUIRE='force')
        run(['ssh',*opts,'-o','NumberOfPasswordPrompts=1','-o','ControlMaster=auto','-o','ControlPersist=600','-MNf',head],env=env,timeout=120)
        run(check,timeout=5);return opts
    def head_python(self,ctx,opts,code):
        return run(['ssh','-T',*opts,'-o','BatchMode=yes','-o','ControlMaster=no',ctx[2],'python3 -c '+shlex.quote(code)],timeout=20)
    def refresh_faces(self):
        def work(ctx):
            result=self.gdk(ctx,{'kind':'faces'})
            return {**result,'message':f"已读取 {len(result['faces'])} 个内置表情"}
        self.launch(work)
    def play_builtin(self):
        path=self.expressions.currentData()
        if not path:self.status.setText('请先点击“刷新表情”读取内置资源');return
        self.gdk_action({'kind':'video','path':path,'loops':-1 if self.loop.isChecked() else 1},'内置表情播放请求已接受')
    def upload_audio(self):
        if self.working:return
        self.operation.disarm()
        filename,_=QtWidgets.QFileDialog.getOpenFileName(self,'选择要播放的音频','','音频文件 (*.mp3 *.wav *.flac *.m4a *.aac *.ogg *.opus *.wma *.aiff)')
        if not filename:return
        volume=self.volume.value()
        def work(ctx):
            opts=self.connect_head(ctx)
            with tempfile.TemporaryDirectory(prefix='g2-audio-') as temp:
                name='audio_'+uuid.uuid4().hex+'.wav';target=Path(temp)/name
                self.progress.emit('正在转换音频：'+Path(filename).name)
                convert_audio(filename,target)
                remote='/home/agi/gdk_resources/collection_audio/'+name
                self.head_python(ctx,opts,"from pathlib import Path;Path('/home/agi/gdk_resources/collection_audio').mkdir(parents=True,exist_ok=True)")
                self.progress.emit('正在上传音频至交互板…')
                run(['scp',*opts,'-o','BatchMode=yes',str(target),ctx[2]+':'+remote+'.partial'],timeout=180)
                self.head_python(ctx,opts,"from pathlib import Path;p=Path(%r);p.replace(%r)"%(remote+'.partial',remote))
                self.progress.emit('音频上传完成，正在请求播放…')
                result=self.gdk(ctx,{'kind':'audio','path':remote,'volume':volume})
            return {**result,'message':'音频已上传，播放请求已接受：'+Path(filename).name}
        self.launch(work)
    def upload(self):
        self.operation.disarm()
        filename,_=QtWidgets.QFileDialog.getOpenFileName(self,'选择表情图片或视频','','图片和视频 (*.png *.jpg *.jpeg *.bmp *.webp *.mp4 *.mov *.mkv *.avi *.webm *.gif)')
        if not filename:return
        loops=-1 if self.loop.isChecked() else 1
        def work(ctx):
            opts=self.connect_head(ctx)
            with tempfile.TemporaryDirectory(prefix='g2-face-') as temp:
                name='face_'+uuid.uuid4().hex+'.mp4';target=Path(temp)/name
                self.progress.emit('正在转换表情素材…');convert_media(filename,target)
                remote='/home/agi/gdk_resources/collection_faces/'+name
                self.head_python(ctx,opts,"from pathlib import Path;Path('/home/agi/gdk_resources/collection_faces').mkdir(parents=True,exist_ok=True)")
                self.progress.emit('正在上传至脸屏交互板…')
                run(['scp',*opts,'-o','BatchMode=yes',str(target),ctx[2]+':'+remote+'.partial'],timeout=180)
                self.head_python(ctx,opts,"from pathlib import Path;p=Path(%r);p.replace(%r)"%(remote+'.partial',remote))
                self.progress.emit('上传完成，正在请求脸屏播放…')
                result=self.gdk(ctx,{'kind':'video','path':remote,'loops':loops})
            return {**result,'message':'表情已上传，播放请求已接受：'+Path(filename).name}
        self.launch(work)
    def done(self,result):
        was_connecting=self.connecting;self.connecting=False;self.working=False;ok,data=result
        if was_connecting and not ok:self.head_key=None;self.connection_status.setText('连接失败')
        if not ok:self.status.setText('操作失败：'+data)
        else:
            if 'connected_key' in data:
                key=data['connected_key']
                if not self.connected() or key!=(self.owner.connected_host,self.head.text().strip()):
                    self.head_key=None;self.connection_status.setText('未连接');self.status.setText('机器人连接已变化，请重新连接 aispeech SSH。');self.update_enabled();return
                self.head_key=key;self.last_check=time.monotonic();self.connection_status.setText('已连接')
            if 'volume' in data:self.volume.setValue(max(0,min(100,data['volume'])))
            if 'faces' in data:
                self.expressions.clear()
                for path in data['faces']:
                    stem=Path(path).stem;self.expressions.addItem(LABELS.get(stem,stem)+' · '+stem,path)
            message=data['message']
            if data.get('confirmed') is True:message+=' · 已确认播放器切换'
            elif data.get('confirmed') is False:message+=' · 尚未确认播放器切换，请查看脸屏'
            self.status.setText(message)
        self.update_enabled()
    def shutdown(self):
        if self.working:
            self.status.setText('正在等待媒体操作完成，请稍后关闭窗口');return False
        self.timer.stop();return True
