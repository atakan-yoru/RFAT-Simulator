import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets, QtCore
import sys
import serial


from Simulator import Target, PathSegment, AntennaPlane, Antenna, DecisionSubsystem, ActuationSubsystem, PathUtils
from Controller import PIDController, KF
from Visualizer import System3DVisualizerPG, ConstellationVisualizerPG, SignalTimeVisualizer, FFTVisualizerPG

from btCom import BluetoothReader
from chirp_tx_jam import chirp_tx_jam as ChirpTopBlock
import test_paths

el_feedback = None
az_feedback = None
el_speed = 0.0
az_speed = 0.0
search_mode = 0
use_motor_feedback = True

def write_az_speed():
    az_data=str(search_mode)+","+str(az_speed)
    return az_data

def write_el_speed():
    el_data=str(search_mode)+","+str(el_speed)
    return el_data

def handle_feedback(axis, value):
    global az_feedback, el_feedback
    if axis == "az":
        az_feedback = value
    elif axis == "el":
        el_feedback = value


EL_PORT = "COM12"  
AZ_PORT = "COM15"  
BLUETOOTH_BAUD = 115200
BLUETOOTH_ENABLE = False


el_serial = None
az_serial = None

bt_az_isconnected=False
bt_el_isconnected=False

while BLUETOOTH_ENABLE and not (bt_az_isconnected and bt_el_isconnected): 
    try:
        if bt_el_isconnected==False:
            el_serial = serial.Serial(EL_PORT, BLUETOOTH_BAUD, timeout=0.1)
            bt_el_isconnected=True
            print(f"[BT EL] Connected to {EL_PORT}")
    except Exception as e:
        bt_el_isconnected=False
        print(f"[BT EL] Failed to connect to {EL_PORT}: {e}")

    try:
        if bt_az_isconnected==False:
            az_serial = serial.Serial(AZ_PORT, BLUETOOTH_BAUD, timeout=0.1)
            bt_az_isconnected=True
            print(f"[BT AZ] Connected to {AZ_PORT}")
    except Exception as e:
        bt_az_isconnected=False
        print(f"[BT AZ] Failed to connect to {AZ_PORT}: {e}")


bt_thread_el = BluetoothReader(el_serial, "el", write_el_speed)
bt_thread_el.feedback_received.connect(handle_feedback)
bt_thread_el.start()

bt_thread_az = BluetoothReader(az_serial, "az", write_az_speed)
bt_thread_az.feedback_received.connect(handle_feedback)
bt_thread_az.start()


def on_exit():
    global search_mode
    search_mode=2
    bt_thread_el.stop()
    bt_thread_az.stop()
    chirp_block.stop()
    chirp_block.wait()
    

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
search_threshold = 1e-3


frame_size = 32
noise_var = 0
SNR = 0
# SNR = 13 #10W için
# SNR = 3 # 1W için
# SNR = -60 # fail için
# SNR = -40 # 1 fail boundary ~ 10 microwatt, thermal noise yok
LARGE_VALUE = 1e6
SMALL_VALUE = 1e-6
search_timeout = 1.0  # 1 second timeout

angle_error_history = []
SNR_rec_history = []


#–– initialize globals so visualization always has something valid


az_err    = np.zeros(frame_size, dtype=np.float32)
el_err    = np.zeros(frame_size, dtype=np.float32)
az_err_norm = np.zeros(frame_size, dtype=np.float32)
el_err_norm = np.zeros(frame_size, dtype=np.float32)
az_filt   = np.zeros(frame_size, dtype=np.float32)
el_filt   = np.zeros(frame_size, dtype=np.float32)
ctrl_az   = np.zeros(frame_size, dtype=np.float32)
ctrl_el   = np.zeros(frame_size, dtype=np.float32)
angle_error_history = [0.0] * 50
rise_time_flag = False
rise_time = 0.0
rise_time_ref = 0.0
setling_time = 0.0
SNR_rec = 0.0
rms_error = 0.0
last_threshold_time = 0.0  # Time when signal first went below threshold
is_below_threshold = False


# --- Target setup ---
base_path = test_paths.test_circle(dt_sim, angular_speed=np.deg2rad(15))
base_path = test_paths.test_6(dt_sim) # Full circle 15

base_path = test_paths.test_7(dt_sim) # Full circle 15->30
base_path = test_paths.test_8(dt_sim) # Full circle elevation change

# base_path = test_paths.test_9(dt_sim) # Complex path eveything changes
# base_path = test_paths.test_10(dt_sim) # Spiral path
base_path = test_paths.test_11(dt_sim) # Elevation change
# base_path = test_paths.test_12(dt_sim) # Search Mode Path

repeated_paths = PathUtils.repeat_paths(base_path, 10)
target = Target(repeated_paths)

# --- Jammer Target setup ---

jammer_path = [
        PathSegment("spherical",
            start=[4, np.deg2rad(45), np.deg2rad(45)],
            end  =[4, np.deg2rad(0), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(0), 0],
            rho_speed=0.0)
    ]
# jammer_path = test_paths.test_1(dt_sim)

jammer_target = Target(jammer_path)

targets = [target, jammer_target]

# --- Antenna setup ---
plane = AntennaPlane(position=(0, 0, 0), orientation=[0, 0, 0])
plane.set_initial_orientation(0,45)

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

pid_az = PIDController(0.9, 0.1, 0.01, frame_size, 1/dt_sim)
pid_el = PIDController(0.9, 0.1, 0.01, frame_size, 1/dt_sim, output_limits=(-np.deg2rad(300), np.deg2rad(300)))


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
rise_time_label = QtWidgets.QLabel("Rise Time: --s")
setling_time_label = QtWidgets.QLabel("Settling Time: --s")
rise_time_ref_label = QtWidgets.QLabel("Rise Time Reference: --s")
rise_time_ref_label.setStyleSheet("color: red; font-weight: bold;")
search_mode_label = QtWidgets.QLabel("Search Mode: 0")
recieved_SNR_label = QtWidgets.QLabel(f"Received SNR: {SNR_rec} dB")
labels_layout.addWidget(rise_time_label)
labels_layout.addWidget(setling_time_label)
labels_layout.addWidget(sim_time_label)
labels_layout.addWidget(angle_error_label)
labels_layout.addWidget(search_mode_label)
labels_layout.addWidget(recieved_SNR_label)
labels_layout.addWidget(rise_time_ref_label)
left_col.addLayout(labels_layout)

# === Right: Use QTabWidget for switching views ===
right_tabs = QtWidgets.QTabWidget()
main_layout.addWidget(right_tabs, stretch=1)

# --- Tab 1: Constellation Plots ---
constellation_tab = QtWidgets.QWidget()
constellation_layout = QtWidgets.QVBoxLayout()
constellation_tab.setLayout(constellation_layout)

vis_meas = ConstellationVisualizerPG("Raw Measurement Errors", point_color=(13, 180, 180))
vis_kf   = ConstellationVisualizerPG("Kalman Filtered Errors", point_color=(38, 115, 255))
vis_ctrl = ConstellationVisualizerPG("Control Outputs", point_color=(255, 115, 38))
constellation_layout.addWidget(vis_meas)
constellation_layout.addWidget(vis_kf)
constellation_layout.addWidget(vis_ctrl)

right_tabs.addTab(constellation_tab, "Constellation")

# --- Tab 2: FFT Visualization ---
fft_tab = QtWidgets.QWidget()
fft_layout = QtWidgets.QVBoxLayout()
fft_tab.setLayout(fft_layout)

# FFT size selector with better visibility in fullscreen
fft_size_label = QtWidgets.QLabel("FFT Size:")
fft_size_label.setStyleSheet("font-size: 14px; color: white;")
fft_layout.addWidget(fft_size_label)

fft_size_combo = QtWidgets.QComboBox()
fft_size_combo.setStyleSheet("QComboBox { min-height: 25px; font-size: 14px; }")
fft_sizes = [2**i for i in range(6, 13)]  # 64 to 4096
for size in fft_sizes:
    fft_size_combo.addItem(str(size))
fft_size_combo.setCurrentText("256")
fft_layout.addWidget(fft_size_combo)

# Create two FFT plots
fft_plot_chirp = FFTVisualizerPG("Chirp Signal FFT", fft_size=256, fs=fs)
fft_plot_noise = FFTVisualizerPG("Noise Signal FFT", fft_size=256, fs=fs)
fft_layout.addWidget(fft_plot_chirp)
fft_layout.addWidget(fft_plot_noise)

right_tabs.addTab(fft_tab, "FFT")
# --- Plot history data ---
time_log, rms_log, az_log, el_log = [], [], [], []
sim_time = 0.0
last_printed_time = -1

chirp_block = ChirpTopBlock(SNR=SNR, frame_size=frame_size)
chirp_block.start()

# --- Simulation and visualization steps ---
def simulation_step():
    sim_start_time = time.time()
    # dt_ms = _sim_timer.restart()
    # print(f"Qt invoked simulation_step every {dt_ms:.1f} ms")
    global sim_time, last_printed_time, az_filt, el_filt, ctrl_az, ctrl_el, az_err, el_err, az_err_norm, el_err_norm
    global el_feedback, el_speed, az_feedback, az_speed, search_mode
    global rise_time_flag, rise_time, setling_time, rms_error, rise_time_ref, SNR_rec, SNR_rec_history
    global is_below_threshold, last_threshold_time

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
        print(f"Target Speed | Azimuth: {np.rad2deg(currnet_segment.angular_speed[0]/currnet_segment.dt  ):.2f}°/s | Elevation: {np.rad2deg(currnet_segment.angular_speed[1]/currnet_segment.dt):.2f}°/s")
        print(f"Search Mode: {search_mode}")

    target.move()
    jammer_target.move()

    angle_error = actuation.compute_error_angle(target)
    angle_error_history.append(angle_error)
    rms_error = np.sqrt(np.mean(np.square(angle_error_history[-50:])))

    if abs(angle_error) <= 10 and not rise_time_flag: 
        rise_time = sim_time-rise_time_ref
        rise_time_flag = True
    elif abs(angle_error) > 10 and rise_time_flag:
        rise_time_ref = sim_time
        rise_time_flag = False

    if rms_error >= 5 :
        setling_time = sim_time-(rise_time_ref+rise_time)

    jam_target_powers = [ant.receive_signal(jammer_target) for ant in plane.antennas]
    target_powers = [ant.receive_signal(target) for ant in plane.antennas]
    
    decision.compute_direction_projection()
    zU, zD, zL, zR = target_powers[0], target_powers[2], target_powers[1], target_powers[3]
    target_distance = target.get_distance()

    JamU, JamD, JamL, JamR = jam_target_powers[0], jam_target_powers[2], jam_target_powers[1], jam_target_powers[3]
    jammer_distance = jammer_target.get_distance()



    I_chirp = np.array(chirp_block.get_I_chirp().level()) / target_distance
    Q_chirp = np.array(chirp_block.get_Q_chirp().level()) / target_distance

    I_noise = np.array(chirp_block.get_I_noise().level()) / jammer_distance
    Q_noise = np.array(chirp_block.get_Q_noise().level()) / jammer_distance


    # print(f"IQ I: {np.mean(iq_I):.2f}, Q: {np.mean(iq_Q):.2f}")
    iq_power_chirp = np.sqrt(I_chirp**2 + Q_chirp**2)
    iq_power_noise = np.sqrt(I_noise**2 + Q_noise**2)
    # print(f"Chirp Power: {np.mean(iq_power_chirp):.2f}, Noise Power: {np.mean(iq_power_noise):.2f}")

    zU_vec = np.full(frame_size, zU * iq_power_chirp) 
    zD_vec = np.full(frame_size, zD * iq_power_chirp) 
    zL_vec = np.full(frame_size, zL * iq_power_chirp) 
    zR_vec = np.full(frame_size, zR * iq_power_chirp)

    jU_vec = np.full(frame_size, JamU * iq_power_noise)
    jD_vec = np.full(frame_size, JamD * iq_power_noise)
    jL_vec = np.full(frame_size, JamL * iq_power_noise)
    jR_vec = np.full(frame_size, JamR * iq_power_noise)

    combU_vec = zU_vec + jU_vec
    combD_vec = zD_vec + jD_vec
    combL_vec = zL_vec + jL_vec
    combR_vec = zR_vec + jR_vec

    SNR_rec = 10 * np.log10((np.mean(zU_vec + zD_vec + zL_vec + zR_vec) / (np.mean(jU_vec + jD_vec + jL_vec + jR_vec))))
    SNR_rec_history.append(SNR_rec)
    SNR_rec = np.mean(SNR_rec_history[-50:])

    combined_signals = np.stack([combU_vec, combD_vec, combL_vec, combR_vec])
    if np.max(combined_signals) < search_threshold:
        if not is_below_threshold:  # First time going below threshold
            last_threshold_time = sim_time
            is_below_threshold = True
        
        if (sim_time - last_threshold_time) >= search_timeout:
            search_mode = 1
    else:
        is_below_threshold = False
        search_mode = 0

    az_ratio = np.clip(combR_vec / (combL_vec + SMALL_VALUE), SMALL_VALUE, LARGE_VALUE)
    el_ratio = np.clip(combU_vec / (combD_vec + SMALL_VALUE), SMALL_VALUE, LARGE_VALUE)
    # print(f"Az Ratio: {np.mean(az_ratio):.2f} | El Ratio: {np.mean(el_ratio):.2f}")

    az_err = np.log10(az_ratio)
    el_err = np.log10(el_ratio)
    # print(f"Azimuth Error: {np.mean(az_err):.2f}° | Elevation Error: {np.mean(el_err):.2f}°")

    az_err = np.nan_to_num(az_err, nan=0.0, posinf=0.0, neginf=0.0)
    el_err = np.nan_to_num(el_err, nan=0.0, posinf=0.0, neginf=0.0)

    az_filt = kf_az.step(az_err)
    el_filt = kf_el.step(el_err)
    az_filt_mean = np.full(frame_size, np.mean(az_filt))
    el_filt_mean = np.full(frame_size, np.mean(el_filt))

    ctrl_az = pid_az.compute_control(az_filt_mean)
    ctrl_el = pid_el.compute_control(el_filt_mean)

    az_speed = -float(np.mean(ctrl_az))
    el_speed = -float(np.mean(ctrl_el))

    if az_feedback is not None and use_motor_feedback:
        az_new = np.deg2rad(az_feedback)
        # print(f"Az Motor Angle: {az_feedback:.2f}°")
    else:
        az_new = plane.current_azimuth + az_speed * dt_sim

    if el_feedback is not None and use_motor_feedback:
        # print(f"El Motor Angle: {el_feedback:.2f}°")
        el_new = np.deg2rad(el_feedback)
    else:
        el_new = plane.current_elevation + el_speed * dt_sim

    # el_new = plane.current_elevation -  el_speed * dt_sim
    plane.update_orientation(az_new, el_new)

    sim_end_time = time.time()
    sim_time_step = (sim_end_time - sim_start_time) * 1000  # Convert to milliseconds
    # print(f"Simulation step took {sim_time_step:.4f} ms")

def visualization_step():
    t0 = time.time()

    # Always update labels and error plot
    sim_time_label.setText(f"Simulation Time: {sim_time:.2f}s")
    angle_error_label.setText(f"RMS Angular Error: {rms_error:.2f}°")
    rise_time_ref_label.setText(f"Rise Time Reference: {rise_time_ref:.2f}s")
    rise_time_label.setText(f"Rise Time: {rise_time:.2f}s")
    setling_time_label.setText(f"Settling Time: {setling_time:.2f}s")
    search_mode_label.setText(f"Search Mode: {search_mode}")
    recieved_SNR_label.setText(f"Received SJNR: {SNR_rec:.2f} dB")

    vis_errors.push_data(sim_time, rms_error, np.mean(ctrl_az), np.mean(ctrl_el))
    
    # Always update 3D visualization
    vis3d_pg.push_state(targets=targets, antennas=plane.antennas, plane=plane, estimated_direction=decision.projection)

    # Update only the active tab's visualizations
    current_tab = right_tabs.currentIndex()
    if current_tab == 0:  # Constellation tab
        vis_meas.push_frame(az_err, el_err)
        vis_kf.push_frame(az_filt, el_filt)
        vis_ctrl.push_frame(ctrl_az, ctrl_el)
    elif current_tab == 1:  # FFT tab
        fft_size = int(fft_size_combo.currentText())
        # Update chirp signal FFT
        chirp_signal = np.array(chirp_block.get_I_chirp().level()) + 1j * np.array(chirp_block.get_Q_chirp().level())
        fft_plot_chirp.push_signal(chirp_signal, fft_size)
        # Update noise signal FFT
        noise_signal = np.array(chirp_block.get_I_noise().level()) + 1j * np.array(chirp_block.get_Q_noise().level())
        fft_plot_noise.push_signal(noise_signal, fft_size)



fft_size_combo.currentTextChanged.connect(lambda: visualization_step())

# --- Timers ---

timer_sim = QtCore.QTimer()
timer_sim.timeout.connect(simulation_step)
timer_sim.start(refresh_sim_ms)

timer_vis = QtCore.QTimer()
timer_vis.timeout.connect(visualization_step)
timer_vis.start(refresh_vis_ms)

main_window.showFullScreen()
def exit_fullscreen(event):
    if event.key() == QtCore.Qt.Key_Escape:
        main_window.showNormal()

main_window.keyPressEvent = exit_fullscreen
app.aboutToQuit.connect(on_exit)
app.exec_()
