import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets, QtCore
import sys


from Simulator import Target, PathSegment, AntennaPlane, Antenna, DecisionSubsystem, ActuationSubsystem
from Controller import PIDController, KF
from Visualizer import System3DVisualizerPG, ConstellationVisualizerPG, SignalTimeVisualizer

from btCom import BluetoothReader
import serial

el_feedback = None
az_feedback = None
el_speed = 0.0
az_speed = 0.0

def handle_feedback(axis, value):
    global az_feedback, el_feedback
    if axis == "az":
        az_feedback = value
    elif axis == "el":
        el_feedback = value


EL_PORT = "COM14"  # COM 12/13
AZ_PORT = "COM12"  # COM 14/15
BLUETOOTH_BAUD = 115200


el_serial = None
az_serial = None

try:
    el_serial = serial.Serial(EL_PORT, BLUETOOTH_BAUD, timeout=0.1)
    print(f"[BT EL] Connected to {EL_PORT}")
except Exception as e:
    print(f"[BT EL] Failed to connect to {EL_PORT}: {e}")

try:
    az_serial = serial.Serial(AZ_PORT, BLUETOOTH_BAUD, timeout=0.1)
    print(f"[BT AZ] Connected to {AZ_PORT}")
except Exception as e:
    print(f"[BT AZ] Failed to connect to {AZ_PORT}: {e}")


bt_thread_el = BluetoothReader(el_serial, "el", lambda: el_speed)
bt_thread_el.feedback_received.connect(handle_feedback)
bt_thread_el.start()

bt_thread_az = BluetoothReader(az_serial, "az", lambda: az_speed)
bt_thread_az.feedback_received.connect(handle_feedback)
bt_thread_az.start()


def on_exit():
    bt_thread_el.stop()
    bt_thread_az.stop()

# & "C:\Program Files\mosquitto\mosquitto.exe" -v
# & "C:\Program Files\mosquitto\mosquitto.exe" -c "D:\Academic\#4th\EE493 & 494\##Simulation\Computer Env\mosquitto.conf" -v
# netstat -aon | findstr :1883
# taskkill /PID 26716 /F

# --- Simulation parameters ---
fs = 25
fps = 30
dt_sim = 1 / fs
dt_vis = 1 / fps
refresh_sim_ms = int(dt_sim * 1000)
refresh_vis_ms = int(dt_vis * 1000)
bt_refresh_ms = 5

use_feeedback = True
frame_size = 64
noise_var = 0
angle_error_history = []

# --- Target path ---

# segments = [
#     # --- Circular sweep at 45° elevation ---
#     PathSegment("spherical",
#                 start=[2, np.deg2rad(0), np.deg2rad(45)],
#                 end  =[2, np.deg2rad(90), np.deg2rad(45)],
#                 dt=dt_sim,
#                 angular_speed=np.deg2rad(60),
#                 rho_speed=0.2),
#     PathSegment("spherical",
#                 start=[0, 0, 0],
#                 end  =[2, np.deg2rad(180), np.deg2rad(45)],
#                 dt=dt_sim,
#                 angular_speed=np.deg2rad(60),
#                 rho_speed=0.2),

#     # --- Linear climb from edge of circular sweep ---
#     PathSegment("linear",
#                 start=[2, 2, 1],
#                 end  =[2, 2, 2.5],
#                 dt=dt_sim,
#                 linear_speed=0.5),

#     # --- Circular sweep at higher elevation ---
#     PathSegment("spherical",
#                 start=[0, 0, 0],
#                 end  =[2, np.deg2rad(270), np.deg2rad(75)],
#                 dt=dt_sim,
#                 angular_speed=np.deg2rad(20),
#                 rho_speed=0.2),

#     # --- Diagonal linear translation ---
#     PathSegment("linear",
#                 start=[-2, -2, 2.5],
#                 end  =[1, 1, 1.5],
#                 dt=dt_sim,
#                 linear_speed=0.7),

#     # --- Low altitude orbit ---
#     PathSegment("spherical",
#                 start=[0, 0, 0],
#                 end  =[1.5, np.deg2rad(360), np.deg2rad(20)],
#                 dt=dt_sim,
#                 angular_speed=np.deg2rad(60),
#                 rho_speed=0.2),

#     # --- Final approach (linear descent) ---
#     PathSegment("linear",
#                 start=[0, 0, 0],
#                 end  =[-2.0, -2.0, 2.0],
#                 dt=dt_sim,
#                 linear_speed=2),

#     PathSegment("linear",
#                 start=[0, 0, 0],
#                 end  =[2.0, 2.0, 2.0],
#                 dt=dt_sim,
#                 linear_speed=1),
# ]


# segments = []

# rho = 2  # Constant radius
# elev = np.deg2rad(45)
# angular_speed = np.deg2rad(60)  # or another value you prefer
# dt = dt_sim
# rho_speed = 0.0  # No radial change

# for i in range(4*10):
#     start_az = np.deg2rad(i * 45)
#     end_az = np.deg2rad((i + 1) * 45)
#     segment = PathSegment(
#         "spherical",
#         start=[rho, start_az, elev],
#         end=[rho, end_az, elev],
#         dt=dt,
#         angular_speed=angular_speed,
#         rho_speed=rho_speed
#     )
#     segments.append(segment)


segments = [
    # --- [1] Quarter circle sweep at 45° elevation (azimuth 0 → 90°) ---
    PathSegment("spherical",
                start=[2, np.deg2rad(0), np.deg2rad(45)],
                end  =[2, np.deg2rad(90), np.deg2rad(45)],
                dt=dt_sim,
                angular_speed=np.deg2rad(30),
                rho_speed=0.0),
    PathSegment("spherical",
                start=[2, np.deg2rad(0), np.deg2rad(45)],
                end  =[2, np.deg2rad(0), np.deg2rad(45)],
                dt=dt_sim,
                angular_speed=np.deg2rad(30),
                rho_speed=0.0),
    PathSegment("spherical",
                start=[2, np.deg2rad(0), np.deg2rad(45)],
                end  =[2, np.deg2rad(90), np.deg2rad(45)],
                dt=dt_sim,
                angular_speed=np.deg2rad(30),
                rho_speed=0.0),

    # --- [2] Elevation flip: 45° → 135° at constant azimuth (az = 90°) ---
    PathSegment("spherical",
                start=[2, np.deg2rad(90), np.deg2rad(45)],
                end  =[2, np.deg2rad(90), np.deg2rad(135)],
                dt=dt_sim,
                angular_speed=np.deg2rad(15),
                rho_speed=0.0),

    # --- [3] Second quarter circle sweep at 135° elevation (azimuth 90 → 180°) ---
    PathSegment("spherical",
                start=[2, np.deg2rad(90), np.deg2rad(135)],
                end  =[2, np.deg2rad(180), np.deg2rad(135)],
                dt=dt_sim,
                angular_speed=np.deg2rad(30),
                rho_speed=0.0),
]



target = Target(segments)

# --- Antenna setup ---
plane = AntennaPlane(position=(0, 0, 0), orientation=[0, 0, 0])
plane.set_initial_orientation(90,0)
# plane.current_elevation = np.deg2rad(45)

tilt = np.deg2rad(25)
poses = [[tilt, 0, 0]]
for _ in range(3):
    poses.append(R.from_euler('z', 90, True).apply(poses[-1]))
for pose in poses:
    ant = Antenna(
        relative_position=[0, 0, 0],
        relative_orientation=pose,
        antenna_plane=plane,
        beamwidth=60
    )
    plane.add_antenna(ant)


decision = DecisionSubsystem(plane)
actuation = ActuationSubsystem(plane, decision)

kf_az = KF(frame_size=frame_size, dt=dt_sim, process_var=1e-1, meas_var=1e-3)
kf_el = KF(frame_size=frame_size, dt=dt_sim, process_var=1e-1, meas_var=1e-3)
# pid_az = PIDController(3.8799, 1.0468, 0.2626, frame_size, 1/dt_sim)
# pid_el = PIDController(3.8799, 1.0468, 0.2626, frame_size, 1/dt_sim, output_limits=(-np.deg2rad(300), np.deg2rad(300)))

pid_az = PIDController(1.5, 0.5, 0.1, frame_size, 1/dt_sim)
pid_el = PIDController(1.5, 0.5, 0.1, frame_size, 1/dt_sim, output_limits=(-np.deg2rad(300), np.deg2rad(300)))


# --- PyQt GUI setup ---
app = QtWidgets.QApplication(sys.argv)
main_window = QtWidgets.QMainWindow()
central_widget = QtWidgets.QWidget()
main_layout = QtWidgets.QHBoxLayout()
central_widget.setLayout(main_layout)
main_window.setCentralWidget(central_widget)
main_window.setWindowTitle("RF Tracking - Unified Visualization")

# === Left: 3D View and Control Plot ===
left_col = QtWidgets.QVBoxLayout()
main_layout.addLayout(left_col, stretch=2)

vis3d_pg = System3DVisualizerPG(mesh_res=12, show_gain_meshes=True)

vis3d_pg.bind_plane(plane)
left_col.addWidget(vis3d_pg, stretch=4)

vis_errors = SignalTimeVisualizer()
left_col.addWidget(vis_errors, stretch=1)


labels_layout = QtWidgets.QHBoxLayout()
angle_error_label = QtWidgets.QLabel("RMS Angular Error: --°")
sim_time_label = QtWidgets.QLabel("Simulation Time: 0.0s")
labels_layout.addWidget(sim_time_label)
labels_layout.addWidget(angle_error_label)
left_col.addLayout(labels_layout)

# === Right: 3x 2D Constellation Plots ===
right_col = QtWidgets.QVBoxLayout()
main_layout.addLayout(right_col, stretch=1)
vis_meas = ConstellationVisualizerPG("Raw Measurement Errors", point_color=(13, 180, 180))
vis_kf   = ConstellationVisualizerPG("Kalman Filtered Errors", point_color=(38, 115, 255))
vis_ctrl = ConstellationVisualizerPG("Control Outputs", point_color=(255, 115, 38))
right_col.addWidget(vis_meas)
right_col.addWidget(vis_kf)
right_col.addWidget(vis_ctrl)

# --- Plot history data ---
time_log, rms_log, az_log, el_log = [], [], [], []
sim_time = 0.0
last_printed_time = -1

# --- Simulation and visualization steps ---
def simulation_step():
    sim_start_time = time.time()

    global sim_time, last_printed_time, az_filt, el_filt, ctrl_az, ctrl_el, az_err, el_err
    global el_feedback, el_speed, az_feedback, az_speed
    sim_time += dt_sim
    if int(sim_time) > last_printed_time:
        last_printed_time = int(sim_time)

        currnet_segment = target.segments[target.current_segment_index]
        current_rho = currnet_segment.current[0]
        current_az = currnet_segment.current[1]
        current_el = currnet_segment.current[2]
        
        print(f"Time: {sim_time:.2f}s | Target | Rho: {current_rho:.2f}m | Azimuth: {np.rad2deg(current_az):.2f}° | Elevation: {np.rad2deg(current_el):.2f}°")
        print(f"Plane | Azimuth: {np.rad2deg(plane.current_azimuth):.2f}° | Elevation: {np.rad2deg(plane.current_elevation):.2f}°")
        print("--------------------------------------------")
        print(f"Control | Azimuth: {np.rad2deg(np.mean(ctrl_az)):.2f}° | Elevation: {np.rad2deg(np.mean(ctrl_el)):.2f}°")
        print(f"Target Speed | Azimuth: {np.rad2deg(currnet_segment.angular_speed/currnet_segment.dt  ):.2f}°/s | Elevation: {np.rad2deg(currnet_segment.angular_speed/currnet_segment.dt):.2f}°/s")

    target.move()
    angle_error = actuation.compute_error_angle(target)
    angle_error_history.append(angle_error)

    for ant in plane.antennas:
        ant.receive_signal(target)

    decision.compute_direction_projection()
    zU, zD, zL, zR = plane.antennas[0].signal_strength, plane.antennas[2].signal_strength, plane.antennas[1].signal_strength, plane.antennas[3].signal_strength
    zU_vec = np.full(frame_size, zU) + np.random.normal(scale=noise_var, size=frame_size)
    zD_vec = np.full(frame_size, zD) + np.random.normal(scale=noise_var, size=frame_size)
    zL_vec = np.full(frame_size, zL) + np.random.normal(scale=noise_var, size=frame_size)
    zR_vec = np.full(frame_size, zR) + np.random.normal(scale=noise_var, size=frame_size)

    az_err = zR_vec - zL_vec
    el_err = zU_vec - zD_vec
    az_filt = kf_az.step(az_err)
    el_filt = kf_el.step(el_err)
    ctrl_az = pid_az.compute_control(np.full(frame_size, np.mean(az_filt)))
    ctrl_el = pid_el.compute_control(np.full(frame_size, np.mean(el_filt)))


    az_speed = float(np.mean(ctrl_az))  
    el_speed = float(np.mean(ctrl_el))

    if az_feedback is not None and use_feeedback:
        print(f"Az Motor Angle: {az_feedback:.2f}°")
        az_new = np.deg2rad(az_feedback)
    else:
        az_new = plane.current_azimuth - az_speed * dt_sim

    if el_feedback is not None and use_feeedback:
        print(f"El Motor Angle: {el_feedback:.2f}°")
        el_new = np.deg2rad(el_feedback)
    else:
        el_new = plane.current_elevation - el_speed * dt_sim

    # el_new = plane.current_elevation -  el_speed * dt_sim
    plane.update_orientation(az_new, el_new)


    sim_end_time = time.time()
    sim_time_step = (sim_end_time - sim_start_time) * 1000  # Convert to milliseconds
    # print(f"Simulation step took {sim_time_step:.4f} ms")

def visualization_step():
    t0 = time.time()

    rms_error = np.sqrt(np.mean(np.square(angle_error_history[-50:])))
    sim_time_label.setText(f"Simulation Time: {sim_time:.2f}s")
    angle_error_label.setText(f"RMS Angular Error: {rms_error:.2f}°")

    vis_errors.push_data(sim_time, rms_error, np.mean(ctrl_az), np.mean(ctrl_el))
    t1 = time.time()


    vis_meas.push_frame(az_err, el_err)
    vis_kf.push_frame(az_filt, el_filt)
    vis_ctrl.push_frame(ctrl_az, ctrl_el)
    t2 = time.time()
    vis3d_pg.push_state(target=target, antennas=plane.antennas, plane=plane, estimated_direction=decision.projection)
    t3 = time.time()
    # print(f"matplotlib: {(t1 - t0)*1000:.2f} ms | 2D scatter: {(t2 - t1)*1000:.2f} ms | 3D: {(t3 - t2)*1000:.2f} ms")


# --- Timers ---
timer_sim = QtCore.QTimer()
timer_sim.timeout.connect(simulation_step)
timer_sim.start(refresh_sim_ms)

timer_vis = QtCore.QTimer()
timer_vis.timeout.connect(visualization_step)
timer_vis.start(refresh_vis_ms)

main_window.show()
app.aboutToQuit.connect(on_exit)
app.exec_()