
from collections import deque
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R
import pyqtgraph as pg

class ConstellationVisualizerPG(QtWidgets.QWidget):
    def __init__(self, title, frame_size=1024, history_frames=5, point_color=(0, 180, 255), refresh_ms=100, xlim=(-10, 10), ylim=(-10, 10)):
        super().__init__()
        self.frame_size = frame_size
        self.history_frames = history_frames
        self.refresh_ms = refresh_ms
        self.point_color = point_color
        self.xlim = xlim
        self.ylim = ylim

        self.history = deque(maxlen=history_frames)
        self.current_frame = np.zeros((0, 2))

        layout = QtWidgets.QVBoxLayout()
        self.label = QtWidgets.QLabel(title)
        layout.addWidget(self.label)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setXRange(*xlim)
        self.plot_widget.setYRange(*ylim)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('bottom', 'Azimuth Error')
        self.plot_widget.setLabel('left', 'Elevation Error')
        layout.addWidget(self.plot_widget)

        self.setLayout(layout)

        self.history_items = []
        for _ in range(history_frames):
            scatter = pg.ScatterPlotItem(pen=None, brush=(150,150,150,80), size=5)
            self.plot_widget.addItem(scatter)
            self.history_items.append(scatter)

        self.current_item = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(*point_color), size=8)
        self.plot_widget.addItem(self.current_item)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(refresh_ms)

    def push_frame(self, azimuth, elevation):
        points = np.column_stack((azimuth, elevation)).astype(np.float32)
        if self.history_frames > 0:
            self.history.appendleft(self.current_frame.copy())
        self.current_frame = points

    def _update_plot(self):
        for i, frame in enumerate(self.history):
            if frame.size > 0:
                self.history_items[i].setData(pos=frame)
            else:
                self.history_items[i].clear()

        if self.current_frame.size > 0:
            self.current_item.setData(pos=self.current_frame)
        else:
            self.current_item.clear()



class System3DVisualizerPG(QtWidgets.QWidget):
    def __init__(self, parent=None, mesh_res=12, show_gain_meshes=True):
        super().__init__(parent)
        self.view = gl.GLViewWidget()
        self.base_axes = self._make_axes(length=1.0)
        for axis in self.base_axes:
            self.view.addItem(axis)
        self.view.setCameraPosition(distance=15)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.view)
        self.home_button = QtWidgets.QPushButton("Home View")
        self.home_button.clicked.connect(self.reset_view)
        layout.addWidget(self.home_button)

        self.setLayout(layout)

        self.mesh_res = mesh_res
        self._init_geometry()
        self.ant_meshes = []
        self.boresights = []
        self.target = None
        self.path = gl.GLLinePlotItem()
        self.view.addItem(self.path)
        self.est_vector = gl.GLLinePlotItem()
        self.view.addItem(self.est_vector)
        
        # --- Prevent NoneType-on-first-paint by giving a tiny default quad ---
        size = 0.5
        dummy_verts = np.array([
            [-size, -size, 0],
            [ size, -size, 0],
            [ size,  size, 0],
            [-size,  size, 0],
        ], dtype=np.float32)
        dummy_faces = np.array([[0,1,2],[0,2,3]], dtype=np.int32)

        self.plane_patch = gl.GLMeshItem(
            vertexes=dummy_verts,
            faces=dummy_faces,
            color=(0.4, 0.2, 1.0, 0.4),
            drawEdges=True,
            smooth=False
        )
        self.plane_normal = gl.GLLinePlotItem()
        self.view.addItem(self.plane_patch)
        self.view.addItem(self.plane_normal)


        self.view.addItem(self.plane_normal)
        self.show_gain_meshes = show_gain_meshes
        

    def _init_geometry(self):
        bw = np.deg2rad(60)
        n = np.log(0.5) / np.log(np.cos(bw / 2))
        t = np.linspace(0, np.pi / 2, self.mesh_res)
        p = np.linspace(0, 2 * np.pi, self.mesh_res)
        T, P = np.meshgrid(t, p)
        Rr = np.cos(T) ** n
        x = (Rr * np.sin(T) * np.cos(P)).ravel()
        y = (Rr * np.sin(T) * np.sin(P)).ravel()
        z = (Rr * np.cos(T)).ravel()
        self._base_pts = np.vstack([x, y, z]).T
        self._mesh_shape = T.shape

        faces = []
        nr, nc = self._mesh_shape
        for i in range(nr - 1):
            for j in range(nc - 1):
                idx = i * nc + j
                faces.append([idx, idx + 1, idx + 1 + nc])
                faces.append([idx, idx + nc, idx + 1 + nc])
        self._faces_idx = np.array(faces)

    def _make_axes(self, length=1.0):
        x_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[length,0,0]]),
                                    color=(1,0,0,1), width=2, mode='lines')
        y_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,length,0]]),
                                    color=(0,1,0,1), width=2, mode='lines')
        z_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,length]]),
                                    color=(0,0,1,1), width=2, mode='lines')
        return [x_axis, y_axis, z_axis]

    
    def reset_view(self):
        self.view.setCameraPosition(distance=15, elevation=30, azimuth=45)


    def bind_plane(self, antenna_plane):
        for ant in antenna_plane.antennas:
            if self.show_gain_meshes:
                dummy = np.zeros_like(self._base_pts)
                mesh = gl.GLMeshItem(vertexes=dummy,
                                    faces=self._faces_idx,
                                    color=(1, 1, 0, 0.3),
                                    drawEdges=True,
                                    drawFaces=True,
                                    smooth=False)
                self.view.addItem(mesh)
                self.ant_meshes.append(mesh)
            else:
                self.ant_meshes.append(None)


            boresight = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 1]]),
                                          color=(1, 0, 0, 1), width=2, mode='lines')
            self.view.addItem(boresight)
            self.boresights.append(boresight)

        # Add target as a sphere
        self.target = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]),
                                           size=10, color=(0, 1, 0, 1))
        self.view.addItem(self.target)

    def push_state(self, target, antennas, plane=None, estimated_direction=None):
        # Boresight
        boresight_color = (1.0, 0.5, 0.0, 1)      # orange
        # Plane normal
        plane_normal_color = (1.0, 0.0, 1.0, 1)   # magenta
        # Plane patch
        plane_patch_color = (0.4, 0.2, 1.0, 0.4)  # semi-transparent
        # Target path
        path_color = (0.8, 0.6, 0.0, 1)           # dark yellow


        for i, ant in enumerate(antennas):
            Rquat = R.from_quat(ant.orientation)
            if self.ant_meshes[i] is not None:
                try:
                    rotated = Rquat.apply(self._base_pts) + ant.position
                    self.ant_meshes[i].setMeshData(vertexes=rotated, faces=self._faces_idx)
                except Exception as e:
                    print(f"[GLMeshItem] Mesh data error for antenna {i}: {e}")
            vec = Rquat.apply([0, 0, 1])
            line = np.array([ant.position, ant.position + vec])
            self.boresights[i].setData(pos=line, color=boresight_color)

        self.target.setData(pos=np.array([target.position]), color=path_color)

        if hasattr(target, 'history') and len(target.history) > 1:
            H = np.array(target.history)
            self.path.setData(pos=H, color=path_color, width=2, mode='line_strip')

        if plane is not None:
            # --- Draw antenna plane surface ---
            size = 0.5  # half-length of plane square
            corners = np.array([
                [-size, -size, 0],
                [ size, -size, 0],
                [ size,  size, 0],
                [-size,  size, 0]
            ])
            rot = R.from_quat(plane.orientation)
            rotated = rot.apply(corners) + plane.origin
            faces = np.array([[0, 1, 2], [0, 2, 3]])
            self.plane_patch.setMeshData(vertexes=rotated, faces=faces, color=plane_patch_color)

            # --- Draw plane normal ---
            normal = rot.apply([0, 0, 1])
            start = plane.origin
            end = start + normal
            self.plane_normal.setData(pos=np.array([start, end]), color=plane_normal_color, width=2, mode='lines')


        if estimated_direction is not None:
            ed = np.array(estimated_direction)
            if np.linalg.norm(ed) != 0:
                ed = ed / np.linalg.norm(ed)
            origin = plane.origin if plane else np.zeros(3)
            self.est_vector.setData(pos=np.array([origin, origin + ed * 2]),
                                    color=(0.5, 0.5, 0.5, 1), width=2, mode='lines')

class SignalTimeVisualizer(QtWidgets.QWidget):
    def __init__(self, max_points=300):
        super().__init__()
        self.max_points = max_points
        self.time = []
        self.rms = []
        self.az = []
        self.el = []

        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        self.plot_rms = pg.PlotWidget(title="RMS Error")
        self.plot_az = pg.PlotWidget(title="Azimuth Control")
        self.plot_el = pg.PlotWidget(title="Elevation Control")

        for plt in [self.plot_rms, self.plot_az, self.plot_el]:
            plt.setBackground('w')
            plt.showGrid(x=True, y=True)
            layout.addWidget(plt)

        self.curves = {
            "rms": self.plot_rms.plot(pen='r'),
            "az": self.plot_az.plot(pen='g'),
            "el": self.plot_el.plot(pen='b')
        }

    def push_data(self, time, rms_val, az_val, el_val):
        self.time.append(time)
        self.rms.append(rms_val)
        self.az.append(az_val)
        self.el.append(el_val)

        # Trim to max points
        if len(self.time) > self.max_points:
            self.time = self.time[-self.max_points:]
            self.rms = self.rms[-self.max_points:]
            self.az = self.az[-self.max_points:]
            self.el = self.el[-self.max_points:]

        self.curves["rms"].setData(self.time, self.rms)
        self.curves["az"].setData(self.time, self.az)
        self.curves["el"].setData(self.time, self.el)
