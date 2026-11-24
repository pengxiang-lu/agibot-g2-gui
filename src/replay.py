"""Synchronized offline video and URDF trajectory replay; never controls the robot."""
import math
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
cv2.setNumThreads(1)
try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ModuleNotFoundError:
    sys.path.append('/usr/lib/python3/dist-packages')
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    finally:
        sys.path.remove('/usr/lib/python3/dist-packages')
try:
    import vtk
    from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
except ModuleNotFoundError:
    sys.path.append('/usr/lib/python3/dist-packages')
    try:
        import vtk
        from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
    finally:
        sys.path.remove('/usr/lib/python3/dist-packages')

ROOT = Path(__file__).resolve().parent.parent
PACKAGE = ROOT/'assets'/'genie_robot_description'
DEFAULT_URDF = PACKAGE/'urdf'/'G2_t2v2_crs_omnipicker.urdf'
CAMERAS = [('head_color','头部相机'),('hand_left_color','左腕相机'),('hand_right_color','右腕相机')]


def xyz(text, default):
    values = [float(v) for v in (text or default).split()]
    if len(values) != 3 or not np.isfinite(values).all(): raise ValueError('URDF 向量必须含三个有限数值')
    return np.asarray(values, dtype=np.float64)


def origin(element):
    matrix = np.eye(4)
    if element is None: return matrix
    x,y,z = xyz(element.get('xyz'),'0 0 0'); roll,pitch,yaw = xyz(element.get('rpy'),'0 0 0')
    cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
    matrix[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    matrix[:3,3]=[x,y,z]
    return matrix


def motion(joint_type,axis,value):
    m=np.eye(4)
    if joint_type=='fixed': return m
    norm=np.linalg.norm(axis)
    if norm<1e-12: raise ValueError('URDF 关节轴长度为 0')
    x,y,z=axis/norm
    if joint_type=='prismatic': m[:3,3]=axis/norm*value;return m
    if joint_type not in ('continuous','revolute'): raise ValueError('不支持的 URDF 关节类型：'+joint_type)
    c,s=np.cos(value),np.sin(value);v=1-c
    m[:3,:3]=[[x*x*v+c,x*y*v-z*s,x*z*v+y*s],
               [y*x*v+z*s,y*y*v+c,y*z*v-x*s],
               [z*x*v-y*s,z*y*v+x*s,z*z*v+c]]
    return m


def vtk_matrix(matrix):
    result=vtk.vtkMatrix4x4()
    for row in range(4):
        for col in range(4): result.SetElement(row,col,float(matrix[row,col]))
    return result


def collada_mesh(path):
    """Read COLLADA geometry and scene matrices used by the supplied G2 gripper."""
    root=ET.parse(path).getroot(); ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    geos={e.get('id'):e for e in root.findall('.//c:library_geometries/c:geometry',ns)}
    points=vtk.vtkPoints(); polys=vtk.vtkCellArray()
    def visit(node,parent):
        transform=parent.copy()
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag=='matrix':
                a=np.fromstring(child.text or '',sep=' ')
                if a.size!=16: raise ValueError('不支持的 COLLADA 变换矩阵')
                transform=transform @ a.reshape(4,4)
            elif tag=='translate':
                a=np.fromstring(child.text or '',sep=' ');t=np.eye(4);t[:3,3]=a;transform=transform@t
            elif tag=='scale':
                a=np.fromstring(child.text or '',sep=' ');t=np.eye(4);t[0,0],t[1,1],t[2,2]=a;transform=transform@t
            elif tag=='rotate':
                a=np.fromstring(child.text or '',sep=' ');transform=transform@motion('revolute',a[:3],math.radians(a[3]))
            elif tag=='instance_geometry':
                geometry=geos.get((child.get('url') or '').lstrip('#'))
                if geometry is None: continue
                mesh=geometry.find('c:mesh',ns)
                sources={}
                for source in mesh.findall('c:source',ns):
                    arr=source.find('c:float_array',ns); accessor=source.find('.//c:accessor',ns)
                    if arr is not None:
                        stride=int(accessor.get('stride','3')) if accessor is not None else 3
                        values=np.fromstring(arr.text or '',sep=' ')
                        if stride>=3 and values.size%stride==0:sources[source.get('id')]=values.reshape(-1,stride)[:,:3]
                vertices={}
                for vertex in mesh.findall('c:vertices',ns):
                    p=vertex.find("c:input[@semantic='POSITION']",ns)
                    if p is not None:vertices[vertex.get('id')]=(p.get('source') or '').lstrip('#')
                for prim in mesh:
                    kind=prim.tag.split('}')[-1]
                    if kind not in ('triangles','polylist'):continue
                    inputs=prim.findall('c:input',ns); refs=[i for i in inputs if i.get('semantic')=='VERTEX']
                    if not refs:continue
                    ref=refs[0];source=sources.get(vertices.get((ref.get('source') or '').lstrip('#')))
                    if source is None:continue
                    stride=max(int(i.get('offset','0')) for i in inputs)+1;offset=int(ref.get('offset','0'))
                    pnode=prim.find('c:p',ns)
                    if pnode is None:continue
                    raw=np.fromstring(pnode.text or '',sep=' ',dtype=np.int64)
                    if raw.size%stride:raise ValueError('COLLADA 面索引格式无效')
                    indices=raw.reshape(-1,stride)[:,offset]
                    counts=np.full(len(indices)//3,3,dtype=int) if kind=='triangles' else np.fromstring(prim.findtext('c:vcount',default='',namespaces=ns),sep=' ',dtype=np.int64)
                    if sum(counts)!=len(indices):raise ValueError('COLLADA 面数量不匹配')
                    transformed=(transform @ np.c_[source,np.ones(len(source))].T).T[:,:3]
                    base=points.GetNumberOfPoints()
                    for p in transformed: points.InsertNextPoint(*map(float,p))
                    pos=0
                    for count in counts:
                        poly=indices[pos:pos+count];pos+=count
                        for i in range(1,len(poly)-1):
                            tri=vtk.vtkTriangle()
                            for n,id_ in enumerate((poly[0],poly[i],poly[i+1])): tri.GetPointIds().SetId(n,base+int(id_))
                            polys.InsertNextCell(tri)
            elif tag=='node':visit(child,transform)
    scene=root.find('.//c:library_visual_scenes/c:visual_scene',ns)
    if scene is None:raise ValueError('COLLADA 缺少 visual_scene')
    for node in scene.findall('c:node',ns):visit(node,np.eye(4))
    if not points.GetNumberOfPoints():raise ValueError('COLLADA 未找到可显示的网格')
    poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(polys)
    normals=vtk.vtkPolyDataNormals();normals.SetInputData(poly);normals.ConsistencyOn();normals.SplittingOff();normals.Update()
    return normals.GetOutput()


class RobotModel:
    def __init__(self,renderer,urdf,joint_names):
        self.renderer=renderer; self.actors=[]; self.joints=[]; self.missing=[]; self.cache={}
        root=ET.parse(urdf).getroot()
        if root.tag!='robot':raise ValueError('所选文件不是 URDF 机器人模型')
        self.links={x.get('name'):x for x in root.findall('link')}
        self.names=set(joint_names);children=set()
        for j in root.findall('joint'):
            parent=j.find('parent').get('link');child=j.find('child').get('link');children.add(child)
            a=j.find('axis');m=j.find('mimic')
            self.joints.append({'name':j.get('name'),'type':j.get('type'),'parent':parent,'child':child,
                'origin':origin(j.find('origin')),'axis':xyz(a.get('xyz') if a is not None else None,'1 0 0'),
                'mimic':(m.get('joint'),float(m.get('multiplier','1')),float(m.get('offset','0'))) if m is not None else None})
        unmatched=self.names-{j['name'] for j in self.joints}
        if unmatched: raise ValueError('URDF 缺少采集关节：'+', '.join(sorted(unmatched)))
        self.root_links=set(self.links)-children
        materials={}
        for m in root.findall('material'):
            c=m.find('color')
            if c is not None: materials[m.get('name')]=[float(x) for x in c.get('rgba').split()]
        for link_name, link in self.links.items():
            for visual in link.findall('visual'):
                geom=visual.find('geometry');mesh=geom.find('mesh') if geom is not None else None
                if mesh is None:continue
                ref=mesh.get('filename',''); path=self.resolve_mesh(ref,Path(urdf))
                if not path or not path.exists(): self.missing.append(ref);continue
                try:
                    poly=self.mesh(path)
                except Exception as exc:
                    self.missing.append(ref+' ('+str(exc)+')');continue
                mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly);mapper.ScalarVisibilityOff()
                actor=vtk.vtkActor();actor.SetMapper(mapper)
                material=visual.find('material');color=material.find('color') if material is not None else None
                rgba=([float(x) for x in color.get('rgba').split()] if color is not None else materials.get(material.get('name'),[0.72,0.78,0.82,1]) if material is not None else [0.72,0.78,0.82,1])
                # Approximate the physical G2's white shell and dark face/grippers.
                # The supplied STL files do not contain per-surface colors.
                if link_name.startswith(('arm_l_link','arm_r_link')): rgba=[.91,.93,.94,1]
                elif link_name == 'head_link3': rgba=[.08,.09,.10,1]
                elif link_name.startswith(('gripper_l_', 'gripper_r_')) and 'camera' not in link_name: rgba=[.24,.25,.26,1]
                actor.GetProperty().SetColor(*rgba[:3]);actor.GetProperty().SetOpacity(rgba[3]);actor.GetProperty().SetSpecular(.2);actor.GetProperty().SetSpecularPower(24)
                scale=xyz(mesh.get('scale'),'1 1 1');s=np.diag([*scale,1.0]);self.actors.append((link_name,actor,origin(visual.find('origin'))@s))
        if not self.actors:raise ValueError('URDF 没有找到可读取的视觉网格，请选择包含网格的模型')
        for _,actor,_ in self.actors:renderer.AddActor(actor)

    def resolve_mesh(self,ref,urdf):
        if ref.startswith('package://genie_robot_description/'):
            rel=ref[len('package://genie_robot_description/'):]
            # Prefer a matching package near the chosen URDF, otherwise use bundled G2 assets.
            for parent in urdf.parents:
                for candidate in (parent/rel,parent/'genie_robot_description'/rel):
                    if candidate.is_file():return candidate
            return PACKAGE/rel
        if ref.startswith('file://'):return Path(ref[7:])
        if ref.startswith('package://'):return None
        return (urdf.parent/ref).resolve()

    def mesh(self,path):
        if path in self.cache:return self.cache[path]
        suffix=path.suffix.lower()
        if suffix=='.stl':
            reader=vtk.vtkSTLReader();reader.SetFileName(str(path));reader.Update();poly=reader.GetOutput()
        elif suffix=='.dae':poly=collada_mesh(path)
        else:raise ValueError('不支持网格类型 '+suffix)
        if poly.GetNumberOfPoints()==0:raise ValueError('网格为空')
        self.cache[path]=poly;return poly

    def update(self,values):
        transforms={name:np.eye(4) for name in self.root_links};pending=self.joints[:]
        while pending:
            remaining=[]
            for j in pending:
                if j['parent'] not in transforms:remaining.append(j);continue
                value=values.get(j['name'],0.0)
                if j['mimic'] and j['name'] not in values:
                    src,mult,offset=j['mimic'];value=values.get(src,0.0)*mult+offset
                transforms[j['child']]=transforms[j['parent']]@j['origin']@motion(j['type'],j['axis'],value)
            if len(remaining)==len(pending):raise ValueError('URDF 关节树不连通或存在循环')
            pending=remaining
        for link,actor,visual_transform in self.actors:
            actor.SetUserMatrix(vtk_matrix(transforms[link]@visual_transform))

    def remove(self):
        for _,actor,_ in self.actors:self.renderer.RemoveActor(actor)


class VideoView(QtWidgets.QLabel):
    def __init__(self):
        super().__init__('该记录未采集此相机')
        self.image=None
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(300,150)
        self.setStyleSheet('background:#10283d;color:#d7e6f4;border-radius:5px;')

    def set_image(self,image):
        self.image=image
        self.rescale()

    def rescale(self):
        if self.image is not None:
            self.setPixmap(QtGui.QPixmap.fromImage(self.image).scaled(
                self.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))

    def resizeEvent(self,event):
        super().resizeEvent(event)
        self.rescale()


class ReplayDialog(QtWidgets.QDialog):
    def __init__(self,path,parent=None):
        super().__init__(parent);self.path=Path(path);self.captures={};self.current=-1;self.playing=False;self.model=None
        self.setWindowTitle('采集可视化 · 三相机与 G2 轨迹');self.resize(1600,950)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.load_dataset();self.build()
        self.load_model(DEFAULT_URDF)
        self.show_frame(0)

    def load_dataset(self):
        import json
        manifest=json.loads((self.path/'capture_manifest.json').read_text(encoding='utf-8'))
        info=json.loads((self.path/'meta/info.json').read_text(encoding='utf-8'))
        self.fps=int(info['fps']); self.joint_names=info['features']['observation.state']['names']
        gripper_feature=info['features'].get('observation.gripper')
        self.gripper_names=list(gripper_feature['names']) if gripper_feature else []
        if self.gripper_names and (len(self.gripper_names)!=2 or len(set(self.joint_names+self.gripper_names))!=len(self.joint_names)+2):raise ValueError('夹爪关节名称无效')
        files=sorted((self.path/'data').rglob('*.parquet'))
        if not files:raise ValueError('未找到关节数据')
        columns=['observation.state','frame_index','episode_index']+(['observation.gripper'] if self.gripper_names else [])
        table=pa.concat_tables([pq.read_table(f,columns=columns) for f in files]).to_pylist()
        if not table:raise ValueError('数据集没有帧')
        episodes={int(r['episode_index']) for r in table}
        if episodes!={0}:raise ValueError('当前回放支持每条记录包含一个 episode 的数据集')
        table.sort(key=lambda r:int(r['frame_index']))
        if [int(r['frame_index']) for r in table]!=list(range(len(table))):raise ValueError('帧序号不连续，无法安全同步')
        self.states=np.asarray([r['observation.state'] for r in table],dtype=np.float64)
        if self.states.shape!=(len(table),len(self.joint_names)) or not np.isfinite(self.states).all():raise ValueError('关节状态数据格式无效')
        self.grippers=np.asarray([r['observation.gripper'] for r in table],dtype=np.float64) if self.gripper_names else None
        if self.grippers is not None and (self.grippers.shape!=(len(table),2) or not np.isfinite(self.grippers).all()):raise ValueError('夹爪数据格式无效')
        self.count=len(table);self.manifest=manifest
        for key,_ in CAMERAS:
            folder=self.path/'videos'/('observation.images.'+key)
            files=sorted(folder.rglob('*.mp4')) if folder.exists() else []
            if len(files)>1:raise ValueError(key+' 视频分块超过一个，当前回放尚不支持')
            if files:
                cap=cv2.VideoCapture(str(files[0]))
                if not cap.isOpened():raise ValueError('无法打开相机视频：'+key)
                frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                if frames and frames!=self.count:cap.release();raise ValueError(f'{key} 视频 {frames} 帧，与状态 {self.count} 帧不一致')
                video_fps=cap.get(cv2.CAP_PROP_FPS)
                if video_fps and abs(video_fps-self.fps)>.02:
                    cap.release();raise ValueError(f'{key} 视频帧率 {video_fps:.3f} 与数据集帧率 {self.fps} 不一致')
                self.captures[key]=cap

    def build(self):
        layout=QtWidgets.QVBoxLayout(self)
        heading=QtWidgets.QHBoxLayout();self.model_label=QtWidgets.QLabel('机器人模型');heading.addWidget(self.model_label,1)
        layout.addLayout(heading)
        split=QtWidgets.QSplitter(QtCore.Qt.Horizontal);layout.addWidget(split,1)
        camera_box=QtWidgets.QWidget();cameras=QtWidgets.QVBoxLayout(camera_box);cameras.setContentsMargins(0,0,0,0)
        self.views={}
        for key,label in CAMERAS:
            group=QtWidgets.QGroupBox(label);box=QtWidgets.QVBoxLayout(group);view=VideoView()
            box.addWidget(view,1);self.views[key]=view;cameras.addWidget(group,1)
        split.addWidget(camera_box)
        model_box=QtWidgets.QGroupBox('G2 关节轨迹（实测状态）');model_layout=QtWidgets.QVBoxLayout(model_box)
        self.vtk_widget=QVTKRenderWindowInteractor(model_box);model_layout.addWidget(self.vtk_widget,1)
        self.renderer=vtk.vtkRenderer();self.renderer.SetBackground(.91,.96,1.0);self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.vtk_widget.GetRenderWindow().SetMultiSamples(0)
        self.interactor=self.vtk_widget.GetRenderWindow().GetInteractor();self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.note=QtWidgets.QLabel('轨迹来自关节状态；未采集的夹爪和底盘关节保持默认姿态。')
        self.note.setWordWrap(True);model_layout.addWidget(self.note);split.addWidget(model_box);split.setSizes([620,980])
        control=QtWidgets.QHBoxLayout();self.play=QtWidgets.QPushButton('▶ 播放');self.play.clicked.connect(self.toggle);control.addWidget(self.play)
        self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.slider.setRange(0,self.count-1);self.slider.sliderPressed.connect(self.pause);self.slider.valueChanged.connect(self.seek);control.addWidget(self.slider,1)
        self.progress=QtWidgets.QLabel();self.progress.setMinimumWidth(210);control.addWidget(self.progress);layout.addLayout(control)
        self.timer=QtCore.QTimer(self);self.timer.setInterval(10);self.timer.timeout.connect(self.tick)
        self.interactor.Initialize()

    def load_model(self,path):
        if not Path(path).is_file():raise ValueError('未找到 G2 URDF：'+str(path))
        model=RobotModel(self.renderer,Path(path),self.joint_names+self.gripper_names)
        if self.model:self.model.remove()
        self.model=model;self.model_label.setText('模型：'+Path(path).name)
        self.note.setText(('轨迹包含左右夹爪实测位置。' if self.grippers is not None else '此记录没有夹爪数据，夹爪保持默认姿态。')+'底盘位姿未采集。'+(f' 缺少或无法加载 {len(model.missing)} 个网格。' if model.missing else ''))
        self.model.update(self.frame_joints(max(0,self.current)))
        self.renderer.ResetCamera()
        cam=self.renderer.GetActiveCamera(); center=cam.GetFocalPoint(); distance=cam.GetDistance()
        cam.SetPosition(center[0]+distance*0.8,center[1]-distance*0.8,center[2]+distance*.45)
        cam.SetViewUp(0,0,1);self.renderer.ResetCameraClippingRange();self.vtk_widget.GetRenderWindow().Render()

    def frame_joints(self,index):
        values=dict(zip(self.joint_names,self.states[index]))
        if self.grippers is not None:values.update(zip(self.gripper_names,self.grippers[index]))
        return values

    def show_frame(self,index):
        index=max(0,min(self.count-1,int(index)))
        for key,cap in self.captures.items():
            if index!=self.current+1:cap.set(cv2.CAP_PROP_POS_FRAMES,index)
            ok,frame=cap.read()
            if not ok:raise RuntimeError(f'{key} 第 {index} 帧无法解码')
            image=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB);h,w=image.shape[:2]
            q=QtGui.QImage(image.data,w,h,int(image.strides[0]),QtGui.QImage.Format_RGB888).copy()
            self.views[key].set_image(q)
        self.current=index;self.model.update(self.frame_joints(index))
        self.renderer.ResetCameraClippingRange();self.vtk_widget.GetRenderWindow().Render()
        self.progress.setText(f'{index+1} / {self.count} 帧   {index/self.fps:.2f} 秒')
        self.slider.blockSignals(True);self.slider.setValue(index);self.slider.blockSignals(False)

    def seek(self,index):
        try:self.show_frame(index)
        except Exception as exc:self.pause();QtWidgets.QMessageBox.critical(self,'回放失败',str(exc))

    def toggle(self):
        if self.playing:self.pause();return
        if self.current>=self.count-1:self.show_frame(0)
        self.playing=True;self.play.setText('❚❚ 暂停');self.start_frame=self.current;self.start_time=time.monotonic();self.timer.start()

    def pause(self):
        self.playing=False;self.timer.stop();self.play.setText('▶ 播放')

    def tick(self):
        index=min(self.count-1,self.start_frame+int((time.monotonic()-self.start_time)*self.fps))
        if index!=self.current:self.seek(index)
        if index>=self.count-1:self.pause()

    def closeEvent(self,event):
        self.pause()
        for cap in self.captures.values():cap.release()
        self.vtk_widget.Finalize();event.accept()
