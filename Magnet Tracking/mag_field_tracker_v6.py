import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from queue import Queue
import struct
from scipy.optimize import least_squares
from sklearn.metrics import r2_score
import socket  # NEW


# Parameters
BAUD_RATE = 921600
NUM_SENSORS_X_PLANE = 18
WINDOW_SIZE = 5  # Moving average window
MAX_NFEV = 20  # Maximum function evaluations for least_squares
NUM_SELECTED_SENSORS = 15  # Number of sensors with highest field to use for tracking

SCALE_ORI = 0.005
DEBUG_ENABLE_TRACKING = True
WARMUP_TIME = 5

# UDP pose streaming (to C++ program on same PC)
POSE_UDP_IP = "127.0.0.1"
POSE_UDP_PORT = 6011  # choose a free port; must match C++ side

# Detection thresholds
R2_THRESHOLD = 0.70       # how good the fit must be to "trust" the solution
MAX_DISTANCE = 0.2       # [m] max allowed |r_opt| before we say "magnet lost"


# Magnetic constants (from aries_magnetic.py)
mag_height = 0.005  # [m]
mag_diam = 0.010  # [m]
mag_const = 1.2 * 1e6  # [A/m], Magnetization of NdFeB magnets
mag_vol = mag_height * np.pi * (mag_diam / 2) ** 2  # [m^3]
mu0 = 4 * np.pi * 1e-7  # [T*m/A]

ENABLE_VISUALIZATION = False  # set to False to disable all matplotlib windows


class DeviceInfo:
    def __init__(self, port_id, com_port, serial_obj, head, tail, num_sensors):
        self.port_id = port_id
        self.com_port = com_port
        self.serial_obj = serial_obj
        self.big_buffer = []
        self.head = head
        self.tail = tail
        self.mag_field_data = np.zeros((num_sensors, 3))
        self.data_queue = Queue()
        self.running = True
        self.lock = threading.Lock()
        # NEW
        self.alive = True
        self.last_sample_time = 0.0  # wall-clock time when we last got a full frame
        self.last_reconnect_attempt = 0.0
        self.error_logged = False


def get_sensor_positions():
    """Define sensor positions in the board local reference frame (in meters)"""
    positions = np.array([
        [87.5, 52.5, -113.27],  # s1
        [52.5, 87.5, -113.27],  # s2
        [17.5, 52.5, -113.27],  # s3
        [52.5, 17.5, -113.27],  # s4
        [17.5, -87.5, -113.27],  # s5
        [52.5, -52.5, -113.27],  # s6
        [17.5, -17.5, -113.27],  # s7
        [87.5, -87.5, -113.27],  # s8
        [87.5, -17.5, -113.27],  # s9
        [-17.5, -52.5, -113.27],  # s10
        [-52.5, -17.5, -113.27],  # s11
        [-87.5, -52.5, -113.27],  # s12
        [-52.5, -87.5, -113.27],  # s13
        [-17.5, 87.5, -113.27],  # s14
        [-52.5, 52.5, -113.27],  # s15
        [-87.5, 87.5, -113.27],  # s16
        [-17.5, 17.5, -113.27],  # s17
        [-87.5, 17.5, -113.27],  # s18
        [-87.5, -114.54, 52.5],  # s19
        [-52.5, -114.54, 87.5],  # s20
        [-17.5, -114.54, 52.5],  # s21
        [-52.5, -114.54, 17.5],  # s22
        [-17.5, -114.54, -87.5],  # s23
        [-52.5, -114.54, -52.5],  # s24
        [-17.5, -114.54, -17.5],  # s25
        [-87.5, -114.54, -87.5],  # s26
        [-87.5, -114.54, -17.5],  # s27
        [17.5, -114.54, -52.5],  # s28
        [52.5, -114.54, -17.5],  # s29
        [87.5, -114.54, -52.5],  # s30
        [52.5, -114.54, -87.5],  # s31
        [17.5, -114.54, 87.5],  # s32
        [52.5, -114.54, 52.5],  # s33
        [87.5, -114.54, 87.5],  # s34
        [17.5, -114.54, 17.5],  # s35
        [87.5, -114.54, 17.5],  # s36
        [-114.03, 87.5, 52.5],  # s37
        [-114.03, 52.5, 87.5],  # s38
        [-114.03, 17.5, 52.5],  # s39
        [-114.03, 52.5, 17.5],  # s40
        [-114.03, 17.5, -87.5],  # s41
        [-114.03, 52.5, -52.5],  # s42
        [-114.03, 17.5, -17.5],  # s43
        [-114.03, 87.5, -87.5],  # s44
        [-114.03, 87.5, -17.5],  # s45
        [-114.03, -17.5, -52.5],  # s46
        [-114.03, -52.5, -17.5],  # s47
        [-114.03, -87.5, -52.5],  # s48
        [-114.03, -52.5, -87.5],  # s49
        [-114.03, -17.5, 87.5],  # s50
        [-114.03, -52.5, 52.5],  # s51
        [-114.03, -87.5, 87.5],  # s52
        [-114.03, -17.5, 17.5],  # s53
        [-114.03, -87.5, 17.5],  # s54
    ]) / 1000.0  # Convert to meters

    return positions


def mag_residual(params, sensor_offsets, B_measured):
    """
    Calculate the residual between measured and predicted magnetic field.
    This is the core function for magnetic tracking.

    Args:
        params: [rx, ry, rz, theta, phi] - position (m) and orientation (rad) of magnet
        sensor_offsets: Nx3 array of sensor positions
        B_measured: Nx3 array of measured magnetic field values (μT)

    Returns:
        Flattened residual array
    """
    rx, ry, rz, theta, phi = params

    # Convert spherical coordinates to Cartesian (ensuring unit vector)
    mx = np.sin(theta) * np.cos(phi)
    my = np.sin(theta) * np.sin(phi)
    mz = np.cos(theta)
    m = np.array([mx, my, mz])

    # Position vector of magnet (dipole) origin, [m]
    r_dipole = np.array([rx, ry, rz])

    # Calculate B field for each sensor
    B_list = []
    for sensor_offset in sensor_offsets:
        r = sensor_offset - r_dipole  # vector from dipole to sensor, [m]
        r_mag = np.linalg.norm(r)  # relative distance from dipole to sensor, [m]

        # Calculate B field components, [T]
        B = ((mag_vol * mag_const * mu0) / (4 * np.pi)) * \
            (((3 * np.dot(m, r) * r) / (r_mag ** 5)) - (m / r_mag ** 3))

        # Conversion of Magnetic field to [μT]
        B = B * 1e4
        B_list.append(B)

    # Flatten the list of B vectors
    B_list = np.array(B_list).flatten()
    B_func = B_measured - B_list

    return B_func


def spherical_to_cartesian(theta, phi):
    """Convert spherical coordinates to Cartesian unit vector"""
    mx_opt = np.sin(theta) * np.cos(phi)
    my_opt = np.sin(theta) * np.sin(phi)
    mz_opt = np.cos(theta)
    return np.array([mx_opt, my_opt, mz_opt])


def assign_port(serial_obj):
    """Assign port ID based on tail marker in the data stream"""
    # Wait long for data to accumulate
    time.sleep(1.5)

    if serial_obj.in_waiting > 0:
        try:
            # Read available bytes
            data = serial_obj.read(serial_obj.in_waiting)

            # Convert bytes to float
            num_floats = len(data) // 4  # 4 bytes per float32
            float_data = []
            if num_floats > 0:
                float_data = struct.unpack(f'<{num_floats}f', data[:num_floats * 4])

            # Check for different tail markers
            if 176 in float_data:
                return 1, 160, 176
            elif 177 in float_data:
                return 2, 161, 177
            elif 178 in float_data:
                return 3, 162, 178
        except Exception as e:
            print(f"Error assigning port: {e}")

    return None, None, None

RECONNECT_INTERVAL = 1.0  # seconds between attempts


def maybe_reconnect(device_info):
    """Try to reopen the same COM port if it died."""
    now = time.time()
    if now - device_info.last_reconnect_attempt < RECONNECT_INTERVAL:
        return

    device_info.last_reconnect_attempt = now

    # Close old handle if needed
    try:
        if device_info.serial_obj is not None and device_info.serial_obj.is_open:
            device_info.serial_obj.close()
    except Exception:
        pass

    try:
        ser = serial.Serial(device_info.com_port, BAUD_RATE, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        time.sleep(0.5)

        device_info.serial_obj = ser
        device_info.big_buffer.clear()
        while not device_info.data_queue.empty():
            device_info.data_queue.get()

        device_info.alive = True
        device_info.error_logged = False
        device_info.last_sample_time = time.time()
        print(f"[RECONNECT] Reconnected device {device_info.port_id} on {device_info.com_port}")

    except Exception as e:
        # Only log once per “failure period” to avoid spam
        if not device_info.error_logged:
            print(f"[RECONNECT] Failed to reconnect {device_info.com_port}: {e}")
            device_info.error_logged = True
        device_info.alive = False


def read_mag_data_threaded(device_info, num_sensors_x_plane):
    """Thread function to continuously read magnetic field data at 500 Hz"""
    read_interval = 1.0 / 500.0  # 500 Hz = 2ms interval

    while device_info.running:
        start_time = time.time()
        # NEW: if port is dead/closed, try to reconnect and skip this cycle
        if device_info.serial_obj is None or (not device_info.serial_obj.is_open):
            device_info.alive = False
            maybe_reconnect(device_info)
            time.sleep(read_interval)
            continue

        try:
            if device_info.serial_obj.in_waiting > 0:
                # Read available data
                buffer = device_info.serial_obj.read(device_info.serial_obj.in_waiting)

                # Convert bytes to floats FIRST (before searching for markers)
                num_floats = len(buffer) // 4
                if num_floats > 0:
                    float_data = struct.unpack(f'<{num_floats}f', buffer[:num_floats * 4])
                    device_info.big_buffer.extend(float_data)

                # Search for tail marker in float buffer
                tail_indices = [i for i, x in enumerate(device_info.big_buffer) if abs(x - device_info.tail) < 0.01]

                if tail_indices:
                    idx_data_end = tail_indices[-1]

                    # Search for head marker before tail
                    head_indices = [i for i, x in enumerate(device_info.big_buffer) if abs(x - device_info.head) < 0.01]
                    valid_heads = [h for h in head_indices if h < idx_data_end]

                    if valid_heads:
                        idx_data_start = valid_heads[-1]

                        # Extract float data between head and tail (already floats!)
                        raw_data = device_info.big_buffer[idx_data_start + 1:idx_data_end]

                        # Check if we have enough float values
                        expected_floats = num_sensors_x_plane * 3
                        if len(raw_data) >= expected_floats:
                            try:
                                # raw_data is already floats, just reshape
                                mag_field_data = np.array(raw_data[:expected_floats]).reshape(num_sensors_x_plane, 3)

                                # Apply coordinate transformations based on port ID
                                with device_info.lock:
                                    if device_info.port_id == 1:
                                        device_info.mag_field_data[:, 0] = mag_field_data[:, 1]
                                        device_info.mag_field_data[:, 1] = -mag_field_data[:, 0]
                                        device_info.mag_field_data[:, 2] = mag_field_data[:, 2]
                                    elif device_info.port_id == 2:
                                        device_info.mag_field_data[:, 0] = -mag_field_data[:, 1]
                                        device_info.mag_field_data[:, 1] = mag_field_data[:, 2]
                                        device_info.mag_field_data[:, 2] = -mag_field_data[:, 0]
                                    elif device_info.port_id == 3:
                                        device_info.mag_field_data[:, 0] = mag_field_data[:, 2]
                                        device_info.mag_field_data[:, 1] = mag_field_data[:, 1]
                                        device_info.mag_field_data[:, 2] = -mag_field_data[:, 0]

                                # Put data in queue for main thread
                                device_info.data_queue.put(device_info.mag_field_data.copy())
                                # Mark this device as alive & fresh
                                device_info.last_sample_time = time.time()
                                device_info.alive = True

                            except Exception:
                                pass

                        # Remove processed data from buffer
                        device_info.big_buffer = device_info.big_buffer[idx_data_end + 1:]

        except Exception as e:
            if not device_info.error_logged:
                print(f"[Serial] Error on {device_info.com_port}: {e}")
                device_info.error_logged = True

            device_info.alive = False
            try:
                device_info.serial_obj.close()
            except Exception:
                pass

        elapsed = time.time() - start_time
        sleep_time = max(0, read_interval - elapsed)
        time.sleep(sleep_time)



def initialize_serial_ports(baud_rate=921600, num_sensors_x_plane=18):
    """Initialize serial communication with acquisition units"""
    available_ports = [port.device for port in serial.tools.list_ports.comports()]
    print(f"Available ports: {available_ports}")

    device_list = []

    for port in available_ports:
        try:
            # Open serial port
            ser = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(0.5)
            ser.reset_input_buffer()
            time.sleep(0.5)
            # ser.reset_input_buffer()

            # Assign port based on tail marker
            port_id, head, tail = assign_port(ser)

            if port_id is not None:
                device = DeviceInfo(port_id, port, ser, head, tail, num_sensors_x_plane)
                device_list.append(device)
                print(f"Port {port} assigned as ID {port_id}")
            else:
                print(f"Failed to assign port {port}")
                ser.close()

        except Exception as e:
            print(f"Error opening port {port}: {e}")

    return device_list


def calibrate_sensors(device_list, num_sensors, num_sensors_x_plane, num_calib_samples=50):
    """Collect calibration data to determine offset"""
    print("Calibrating sensors...")

    calib_buffer = np.zeros((num_sensors, 3, num_calib_samples))
    sample_count = np.zeros(len(device_list), dtype=int)

    while np.min(sample_count) < num_calib_samples:
        for idx, device in enumerate(device_list):
            if device.serial_obj.in_waiting > 0:
                device.serial_obj.reset_input_buffer()
                time.sleep(0.01)

                # Read and process data as floats
                if device.serial_obj.in_waiting > 0:
                    num_bytes = device.serial_obj.in_waiting
                    buffer = device.serial_obj.read(num_bytes)

                    # Convert bytes to float32 array
                    num_floats = len(buffer) // 4
                    if num_floats > 0:
                        float_data = struct.unpack(f'<{num_floats}f', buffer[:num_floats * 4])
                        device.big_buffer.extend(float_data)

                    # Search for tail marker in float buffer
                    tail_indices = [i for i, x in enumerate(device.big_buffer) if abs(x - device.tail) < 0.01]
                    if tail_indices:
                        idx_data_end = tail_indices[-1]
                        head_indices = [i for i, x in enumerate(device.big_buffer) if abs(x - device.head) < 0.01]
                        valid_heads = [h for h in head_indices if h < idx_data_end]

                        if valid_heads and sample_count[idx] < num_calib_samples:
                            idx_data_start = valid_heads[-1]
                            raw_data = device.big_buffer[idx_data_start + 1:idx_data_end]

                            expected_floats = num_sensors_x_plane * 3
                            if len(raw_data) >= expected_floats:
                                try:
                                    # raw_data is already floats, just reshape
                                    mag_data = np.array(raw_data[:expected_floats]).reshape(num_sensors_x_plane, 3)

                                    # Apply coordinate transformations based on port ID (same as in reading function)
                                    transformed_data = np.zeros((num_sensors_x_plane, 3))
                                    if device.port_id == 1:
                                        transformed_data[:, 0] = mag_data[:, 1]
                                        transformed_data[:, 1] = -mag_data[:, 0]
                                        transformed_data[:, 2] = mag_data[:, 2]
                                    elif device.port_id == 2:
                                        transformed_data[:, 0] = -mag_data[:, 1]
                                        transformed_data[:, 1] = mag_data[:, 2]
                                        transformed_data[:, 2] = -mag_data[:, 0]
                                    elif device.port_id == 3:
                                        transformed_data[:, 0] = mag_data[:, 2]
                                        transformed_data[:, 1] = mag_data[:, 1]
                                        transformed_data[:, 2] = -mag_data[:, 0]

                                    sensor_start = num_sensors_x_plane * (device.port_id - 1)
                                    sensor_end = num_sensors_x_plane * device.port_id
                                    calib_buffer[sensor_start:sensor_end, :, sample_count[idx]] = transformed_data
                                    sample_count[idx] += 1
                                    print(
                                        f"Calibration progress: Device {device.port_id} - {sample_count[idx]}/{num_calib_samples}")
                                except:
                                    pass

                            device.big_buffer = device.big_buffer[idx_data_end:]

    offset = np.mean(calib_buffer[:, :, :num_calib_samples], axis=2)

    print("\n" + "=" * 70)
    print("CALIBRATION COMPLETE!")
    print("=" * 70)

    # Print offset for each device (18x3 matrix)
    for device in device_list:
        sensor_start = num_sensors_x_plane * (device.port_id - 1)
        sensor_end = num_sensors_x_plane * device.port_id
        device_offset = offset[sensor_start:sensor_end, :]

        print(f"\nDevice {device.port_id} ({device.com_port}) - Offset Matrix [{num_sensors_x_plane}x3]:")
        print("-" * 70)
        print(f"{'Sensor':<8} {'X-axis':>12} {'Y-axis':>12} {'Z-axis':>12}")
        print("-" * 70)

        for i in range(num_sensors_x_plane):
            sensor_num = sensor_start + i + 1  # Sensor numbering starts at 1
            print(
                f"s{sensor_num:<7} {device_offset[i, 0]:>12.2f} {device_offset[i, 1]:>12.2f} {device_offset[i, 2]:>12.2f}")

        print("-" * 70)
        print(
            f"{'Mean':<8} {np.mean(device_offset[:, 0]):>12.2f} {np.mean(device_offset[:, 1]):>12.2f} {np.mean(device_offset[:, 2]):>12.2f}")
        print(
            f"{'Std Dev':<8} {np.std(device_offset[:, 0]):>12.2f} {np.std(device_offset[:, 1]):>12.2f} {np.std(device_offset[:, 2]):>12.2f}")

    print("=" * 70 + "\n")

    return offset


def main():
    # Initial guess for tracking: [x, y, z, theta, phi]
    # Position in meters, angles in radians
    default_params = np.array([0.0, 0.0, 0.0, np.pi / 2, 0.0])
    initial_params = default_params.copy()
    m_init = spherical_to_cartesian(initial_params[3], initial_params[4])

    # Initialize tracking variables
    window_results = np.zeros((WINDOW_SIZE, 5))
    window_results_index = 0
    magnet_found = False

    # Statistics tracking
    tracking_stats = {
        'r_squared': [],
        'distance': [],
        'ls_time': [],
        'success_count': 0,
        'fail_count': 0
    }

    # Get sensor positions
    sensor_positions = get_sensor_positions()
    num_sensors = len(sensor_positions)

    # Initialize serial ports
    device_list = initialize_serial_ports(BAUD_RATE, NUM_SENSORS_X_PLANE)

    if not device_list:
        print("No devices found!")
        return

    if len(device_list) < 3:
        print(f"\n⚠ Error: Only {len(device_list)} device(s) connected, expected 3")
        return

    # ========== FLUSH ALL BUFFERS ==========
    print("\nFlushing serial buffers...")
    for device in device_list:
        # Clear the big_buffer that was used during port assignment
        device.big_buffer.clear()

        # Flush both input and output buffers
        device.serial_obj.reset_input_buffer()  # Flush RX buffer
        device.serial_obj.reset_output_buffer()  # Flush TX buffer

        # Read and discard any remaining data
        if device.serial_obj.in_waiting > 0:
            _ = device.serial_obj.read(device.serial_obj.in_waiting)

        print(f"  ✓ Device {device.port_id} buffers flushed")

    # Wait a moment to ensure buffers are clear
    time.sleep(0.1)
    print("✓ All buffers cleared\n")

    # Calibrate sensors
    mag_field_offset = calibrate_sensors(device_list, num_sensors, NUM_SENSORS_X_PLANE)

    # Start reading threads for each device
    threads = []
    for device in device_list:
        thread = threading.Thread(target=read_mag_data_threaded, args=(device, NUM_SENSORS_X_PLANE))
        thread.daemon = True
        thread.start()
        threads.append(thread)

        # Initialize quiver plot for magnetic field vectors
        mag_field_data = np.zeros((num_sensors, 3))

        if ENABLE_VISUALIZATION:
            print("Starting real-time visualization at 30 Hz...")

            # Setup 3D plot
            fig = plt.figure(figsize=(14, 10))
            ax = fig.add_subplot(111, projection='3d')

            # Plot sensor positions
            scatter = ax.scatter(sensor_positions[:, 0], sensor_positions[:, 1],
                                sensor_positions[:, 2], c='blue', marker='o', s=50, alpha=0.6)

            if DEBUG_ENABLE_TRACKING:
                m_init = spherical_to_cartesian(initial_params[3], initial_params[4])
                scatter_mag = ax.scatter(initial_params[0], initial_params[1],
                                        initial_params[2], c='black', marker='o', s=50, alpha=1)
                scatter_magN = ax.scatter(initial_params[0] + SCALE_ORI * m_init[0],
                                        initial_params[1] + SCALE_ORI * m_init[1],
                                        initial_params[2] + SCALE_ORI * m_init[2], c='red', marker='o', s=15, alpha=1)
                scatter_magS = ax.scatter(initial_params[0] - SCALE_ORI * m_init[0],
                                        initial_params[1] - SCALE_ORI * m_init[1],
                                        initial_params[2] - SCALE_ORI * m_init[2], c='blue', marker='o', s=15, alpha=1)

            # Initialize with zero field
            b_field = np.zeros((num_sensors, 3))
            quiver = ax.quiver(sensor_positions[:, 0], sensor_positions[:, 1], sensor_positions[:, 2],
                            b_field[:, 0], b_field[:, 1], b_field[:, 2],
                            color='red', arrow_length_ratio=0.3, linewidth=2, alpha=0.8)

            ax.set_xlabel('X axis (m)', fontsize=10)
            ax.set_ylabel('Y axis (m)', fontsize=10)
            ax.set_zlabel('Z axis (m)', fontsize=10)
            ax.set_title('Magnetic Field Visualization (30 Hz Update)', fontsize=12, fontweight='bold')
            ax.view_init(elev=20, azim=45)

            # Set equal aspect ratio
            max_range = np.array([sensor_positions[:, 0].max() - sensor_positions[:, 0].min(),
                                sensor_positions[:, 1].max() - sensor_positions[:, 1].min(),
                                sensor_positions[:, 2].max() - sensor_positions[:, 2].min()]).max() / 2.0
            mid_x = (sensor_positions[:, 0].max() + sensor_positions[:, 0].min()) * 0.5
            mid_y = (sensor_positions[:, 1].max() + sensor_positions[:, 1].min()) * 0.5
            mid_z = (sensor_positions[:, 2].max() + sensor_positions[:, 2].min()) * 0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

            # Add text for statistics
            stats_text = ax.text2D(0.02, 0.98, '', transform=ax.transAxes,
                                verticalalignment='top', fontsize=9, family='monospace',
                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            plt.ion()
            plt.show()
        else:
            print("Visualization DISABLED – running headless (UDP pose only).")


    # ========== FINAL BUFFER FLUSH BEFORE MAIN LOOP ==========
    print("Final buffer flush before starting main loop...")
    for device in device_list:
        # Clear accumulated data from calibration period
        device.big_buffer.clear()

        # Clear the data queue
        while not device.data_queue.empty():
            device.data_queue.get()

        # Flush serial buffers one more time
        device.serial_obj.reset_input_buffer()
        if device.serial_obj.in_waiting > 0:
            _ = device.serial_obj.read(device.serial_obj.in_waiting)

    # Wait for buffers to stabilize with fresh data
    #time.sleep(0.2)
    print("✓ Buffers cleared, starting with fresh data\n")

    print("Entering the While Loop")

    # Visualization parameters
    display_interval = 1.0 / 30.0
    last_display_time = time.time()
    frame_count = 0
    start_time = time.time()

    # UDP socket to send pose to C++ host
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # --- frequency measurement ---
    freq_last_print = time.time()
    pose_update_count = 0     # counts every UDP pose send
    ls_update_count = 0       # counts every LS solve (tracking step)

    try:
        # Main visualization loop
        while True:
            current_time = time.time()

            # Continuously collect data from all devices
            for device in device_list:
                # Get latest data (drain queue to get freshest)
                latest_data = None
                while not device.data_queue.empty():
                    latest_data = device.data_queue.get()

                if latest_data is not None:
                    sensor_start = NUM_SENSORS_X_PLANE * (device.port_id - 1)
                    sensor_end = NUM_SENSORS_X_PLANE * device.port_id
                    mag_field_data[sensor_start:sensor_end, :] = latest_data

            # Apply offset to get calibrated field
            b_field_calibrated = (mag_field_data - mag_field_offset) / 100.0
            MATRIX_TIMEOUT = 0.05  # 50 ms; allow ~1–2 missed samples at 500 Hz

            now = current_time  # already computed above

            matrix_ok = True
            for dev in device_list:
                # stale or explicitly marked dead?
                if (not dev.alive) or (now - dev.last_sample_time > MATRIX_TIMEOUT):
                    matrix_ok = False
                    break

            # >>> NEW: if matrix is unhealthy, zero the field map for GUI + stats
            if not matrix_ok:
                b_field_calibrated[:] = 0.0


            if DEBUG_ENABLE_TRACKING and (current_time - start_time) >= WARMUP_TIME:
                # ========== MAGNETIC TRACKING ==========

                # Calculate field magnitudes for each sensor
                field_magnitudes = np.linalg.norm(b_field_calibrated, axis=1)

                # Get indices of sensors with largest field magnitudes
                top_sensor_indices = np.argsort(field_magnitudes)[-NUM_SELECTED_SENSORS:]

                # Select corresponding sensor positions and field measurements
                selected_positions = sensor_positions[top_sensor_indices]
                selected_field = b_field_calibrated[top_sensor_indices]

                # Prepare data for least squares optimization
                B_measured = selected_field.flatten()

                # Run least squares optimization with selected sensors only
                ls_start = time.time()
                result = least_squares(
                    mag_residual,
                    initial_params,
                    max_nfev=MAX_NFEV,
                    args=(selected_positions, B_measured)
                )
                ls_time = time.time() - ls_start
                ls_update_count += 1

                # Calculate R² for quality assessment
                B_predicted = B_measured - result.fun
                r_squared = r2_score(B_measured, B_predicted)

                tracking_stats['r_squared'].append(r_squared)
                tracking_stats['ls_time'].append(ls_time)

                # Extract parameters
                mag_x, mag_y, mag_z, theta, phi = result.x

                # ---- Detection logic: do we trust this solution? ----
                # (we will only update the filtered pose + send non-zero data if True)
                # NOTE: we compute r_opt ONLY from the raw result.x, then decide.
                r_candidate = np.array(result.x[:3])
                pred_distance = np.linalg.norm(r_candidate)
                tracking_stats['distance'].append(pred_distance)

                magnet_detected = (r_squared >= R2_THRESHOLD) and (pred_distance <= MAX_DISTANCE)

                if magnet_detected:
                    magnet_found = True

                    # Update moving average window
                    window_results[window_results_index] = result.x
                    window_results_index = (window_results_index + 1) % WINDOW_SIZE

                    # Calculate filtered results
                    filtered_results = np.average(window_results, axis=0)
                    std_results = np.std(window_results[:, :3], axis=0)

                    # Get optimized position and orientation
                    r_opt = filtered_results[:3]
                    m_opt = spherical_to_cartesian(filtered_results[3], filtered_results[4])
                    pretty_angle = np.arccos(np.clip(abs(m_opt[2]), -1.0, 1.0))  # radians
                    pretty_deg = pretty_angle * 180.0 / np.pi

                    # --- send real pose to C++ over UDP ---
                    payload = struct.pack(
                        "<6f",
                        float(r_opt[0]), float(r_opt[1]), float(r_opt[2]),
                        float(m_opt[0]), float(m_opt[1]), float(m_opt[2])
                    )
                    pose_sock.sendto(payload, (POSE_UDP_IP, POSE_UDP_PORT))
                    pose_update_count += 1
                    # Update initial params for next iteration (good convergence point)
                    initial_params = result.x

                else:
                    # Magnet lost / out of range → mark as not found, send zeros
                    magnet_found = False
                    pretty_deg = 0.0  # optional, just for display
                    r_opt = np.array([0.0, 0.0, 0.0])
                    m_opt = np.array([0.0, 0.0, 1.0])  # arbitrary "up" vector

                    # Send zeros to the C++ side
                    payload = struct.pack(
                        "<6f",
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0
                    )
                    pose_sock.sendto(payload, (POSE_UDP_IP, POSE_UDP_PORT))
                    pose_update_count += 1
                    # IMPORTANT: do NOT update initial_params from a bad solution
                    # so that LS can re-lock from the last good state / default.

            else:
                # Matrix unhealthy OR still warming up OR tracking disabled
                magnet_found = False
                pretty_deg = 0.0
                r_opt = np.array([0.0, 0.0, 0.0])
                m_opt = np.array([0.0, 0.0, 1.0])

                payload = struct.pack(
                    "<6f",
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0
                )
                pose_sock.sendto(payload, (POSE_UDP_IP, POSE_UDP_PORT))
                pose_update_count += 1
            # Update display at 30 Hz
            if ENABLE_VISUALIZATION and (current_time - last_display_time >= display_interval):

                # ========== VISUALIZATION UPDATE ==========

                # Calculate magnitudes for statistics
                magnitudes = np.linalg.norm(b_field_calibrated, axis=1)

                # Scale factor for arrow visualization (adjust as needed)
                # This makes arrows proportional to field magnitude
                arrow_scale = 0.02  # Adjust this to make arrows bigger/smaller
                scaled_field = b_field_calibrated * arrow_scale

                # Update quiver plot with magnitude-based scaling
                quiver.remove()

                # Color arrows based on magnitude
                colors = plt.cm.plasma(magnitudes / (np.max(magnitudes) + 1e-10))

                quiver = ax.quiver(sensor_positions[:, 0], sensor_positions[:, 1], sensor_positions[:, 2],
                                   scaled_field[:, 0], scaled_field[:, 1], scaled_field[:, 2],
                                   colors=colors, arrow_length_ratio=0.3, linewidth=2, alpha=0.8)

                if DEBUG_ENABLE_TRACKING and (current_time - start_time) >= WARMUP_TIME and matrix_ok:
                    scatter_mag.remove()
                    scatter_magN.remove()
                    scatter_magS.remove()

                    scatter_mag = ax.scatter(initial_params[0], initial_params[1],
                                             initial_params[2], c='black', marker='o', s=50, alpha=1)
                    scatter_magN = ax.scatter(initial_params[0] + SCALE_ORI * m_opt[0],
                                              initial_params[1] + SCALE_ORI * m_opt[1],
                                              initial_params[2] + SCALE_ORI * m_opt[2], c='red', marker='o', s=15,
                                              alpha=1)
                    scatter_magS = ax.scatter(initial_params[0] - SCALE_ORI * m_opt[0],
                                              initial_params[1] - SCALE_ORI * m_opt[1],
                                              initial_params[2] - SCALE_ORI * m_opt[2], c='blue', marker='o', s=15,
                                              alpha=1)

                # Update statistics text
                frame_count += 1
                fps = frame_count / (current_time - start_time)

                # Before stats_info construction
                if matrix_ok:
                    matrix_status = "MATRIX: OK"
                else:
                    matrix_status = "MATRIX: DISCONNECTED/STALE"
                if DEBUG_ENABLE_TRACKING and (current_time - start_time) >= WARMUP_TIME:
                    

                    stats_info = (
                        f"{matrix_status}\n"
                        f"Display FPS: {fps:.1f}\n"
                        f"Field Magnitude (μT):\n"
                        f"  Min:  {np.min(magnitudes):.3f}\n"
                        f"  Max:  {np.max(magnitudes):.3f}\n"
                        f"  Mean: {np.mean(magnitudes):.3f}\n"
                        f"  Std:  {np.std(magnitudes):.3f}\n"
                        f"\n=== MAGNET PARAMS ===\n"
                        f"X: {r_opt[0]:>7.4f} m\n"
                        f"Y: {r_opt[1]:>7.4f} m\n"
                        f"Z: {r_opt[2]:>7.4f} m\n"
                        f"Angle w.r.t Z-axis: {pretty_deg:>6.1f}°"
                    )
                else:
                    stats_info = (
                        f"{matrix_status}\n"
                        f"Display FPS: {fps:.1f}\n"
                        f"Field Magnitude (μT):\n"
                        f"  Min:  {np.min(magnitudes):.3f}\n"
                        f"  Max:  {np.max(magnitudes):.3f}\n"
                        f"  Mean: {np.mean(magnitudes):.3f}\n"
                        f"  Std:  {np.std(magnitudes):.3f}"
                    )

                stats_text.set_text(stats_info)

                plt.draw()
                plt.pause(0.001)  # Minimal pause to update display

                last_display_time = current_time
            else:
                # Sleep a bit to avoid busy waiting
                time.sleep(0.001)
            # --- Frequency printout every ~1 s ---
            now_freq = time.time()
            dt = now_freq - freq_last_print
            if dt >= 1.0:
                pose_hz = pose_update_count / dt
                ls_hz   = ls_update_count / dt
                print(f"[FREQ] pose sends: {pose_hz:.1f} Hz, LS solves: {ls_hz:.1f} Hz")
                freq_last_print = now_freq
                pose_update_count = 0
                ls_update_count   = 0

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Clean up
        for device in device_list:
            device.running = False
            device.serial_obj.close()

        for thread in threads:
            thread.join(timeout=1)

        if ENABLE_VISUALIZATION:
            plt.close()
        print("Program terminated.")



if __name__ == "__main__":
    main()
