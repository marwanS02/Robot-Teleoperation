#!/usr/bin/env python3
import time
import numpy as np
import os
import sys
from datetime import datetime
from scipy.optimize import least_squares
from sklearn.metrics import r2_score
import argparse
from geometry_msgs.msg import PoseStamped
import rospy
import threading

from std_msgs.msg import Float64MultiArray

from franka_msgs.msg import FrankaState


# Add the path to custom libraries
path = os.path.dirname(os.path.abspath(__file__))
norm_path = os.path.normpath(path + '/../../py_libraries')
sys.path.insert(1, norm_path)
import aries_magnetic as am
import utils

# Constants for serial communication and data collection
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
CALIBRATION_TIME = 180
RECORDING_TIME = 600
GOAL_CALIB = 0.9
MAGNET_POS_W = np.array([0.54788, 0.00006, 0.32819])
# Directories and filenames for data storage
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
FOLDER_NAME = "magnetic_data"
BASE_FILENAME = 'rawsensor'
BASE_FILENAME_POSE = 'eeposes'
BASE_FILENAME_HOM = 'homogen'
BASE_FILENAME_EEMAG_POS = 'relativemagpos'
BASE_FILENAME_VALUES = 'cleansensor'
BASE_FILENAME_NORDAV = 'nordav'
MESH_PATH = os.path.join(CURRENT_DIR, 'magnetic_data/mountedvision.stl')

low_R2 = 0.4
high_R2 = 0.9
mask_axis = [1,0,0]
r_sphere = 0.30 #[m]
gripp_orient = np.array((9.99999994e-01, -8.02161440e-05,  9.66399028e-06, -6.81021850e-05))
start_pos = np.hstack((np.array([0.5,0,0.5]),gripp_orient))


# Static homogeneous transformation matrix
det_H_link8 = np.array([[0,                        0,   1,  -0.035],
                        [-1/np.sqrt(2), -1/np.sqrt(2),  0,  0],
                        [1/np.sqrt(2),  -1/np.sqrt(2),  0,  -0.094],
                        [0,             0,              0,  1]
])


L8_H_HAND =          np.array([[0.707,     0.707,  0,  0],
                      [-0.707,    0.707,  0,  0],
                      [0,         0,      1,  0.1034],
                      [0,         0,      0,  1]]  )

DET_H_HAND = np.dot(det_H_link8, L8_H_HAND)


HAND_h_L8 =np.linalg.inv(L8_H_HAND)


HAND_H_DET = np.linalg.inv(DET_H_HAND)
# Extract rotation matrix and compute inverse transformation
det_R_link8 = det_H_link8[:3, :3]
link8_H_det = np.linalg.inv(det_H_link8)
link8_R_det = link8_H_det[:3, :3]
DET_R_HAND = DET_H_HAND[:3, :3]
HAND_R_DET =HAND_H_DET[:3, :3]
# Sensor offsets for model 3 plate in meters
sensor_offsets = [
    [0, -0.07760, -0.05350],
    [0, -0.02500, 0.00760],
    [0, 0.02500, 0.00760],
    [0, 0.07760, -0.05350]
]

def setup_robot_moveit():
    import aries_franka_control as afc
    """Set up the robot if it's connected."""
    
    # Set up end effector
    ee = afc.EndEffectorType.AWL
    # Initialize robot controller
    fr3 = afc.RobotController(ee)
    fr3.create_scene_from_json("aries_gazebo_workspace.json")
    
    # Create the publisher here
    mag_pos_pub = rospy.Publisher('/magnet_position', PoseStamped, queue_size=1)
    
    return fr3, utils, mag_pos_pub

W_H_EE = np.zeros((4,4))
robot_pose_lock = threading.Lock()
def franka_states_cb(franka_states_ms:FrankaState):
    global W_H_EE
    robot_pose_lock.acquire()
    W_H_EE[:,:] = np.reshape(franka_states_ms.O_T_EE, (4,4)).T # NOTE: prova
    robot_pose_lock.release()

def setup_robot():
    import aries_franka_control as afc
    import utils
    """Set up the robot if it's connected."""

    franka_state_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, callback=franka_states_cb, queue_size=1)
    
    # Create the publisher here
    mag_pos_pub = rospy.Publisher('/magnet_position', PoseStamped, queue_size=1)
    sensor_values_pub = rospy.Publisher('/magnet_sensor_values', Float64MultiArray, queue_size=1)
    
    return  utils, mag_pos_pub, sensor_values_pub

def get_robot_current_pose():
    global W_H_EE

    return 0


def process_nord_data(mag, calib_data, nord_duration=5):
    """Process and average the north vector data."""
    print("Averaging the sensed nord.... Make sure you have no potential sources of magnetic field nearby and do not move the robot!")
    nord_arr = None
    start_time = time.time()
    while time.time() - start_time < nord_duration:
        data_line = mag.receive_sensor_data_line(calib_data)
        if data_line is not None:
            nord_arr = data_line if nord_arr is None else np.dstack((nord_arr, data_line))
        elapsed_time = time.time() - start_time
        mag.percentage_update("Sensing north...", elapsed_time / nord_duration)

    # Compute the average along the depth to get an averaged data_line matrix
    return np.average(nord_arr, axis=2)

def main():
    parser = argparse.ArgumentParser(description='Magnet position sensing and publishing.')
    parser.add_argument('--debug', action='store_true', help='Run in debug mode without publishing to the robot')
    args = parser.parse_args()

    # Create folder for saving data
    folder_path = os.path.join(CURRENT_DIR, FOLDER_NAME)
    os.makedirs(folder_path, exist_ok=True)

    # Initialize the aries_magnetic class
    mag = am.aries_magnetic(SERIAL_PORT, BAUD_RATE, CALIBRATION_TIME)
    

    # Check if robot is connected
    with_robot = mag.answer("Is the robot connected? (y/n): ")    
    utils = None
    mag_pos_pub = None

    # Initialize ROS node
    rospy.init_node('FrankaMainMagSensing', anonymous=True, disable_signals=True)
    
    if with_robot:
        utils, mag_pos_pub, sensor_values_pub = setup_robot()
    else:
        # If not using the robot, initialize a node for publishing
        mag_pos_pub = rospy.Publisher('/magnet_position', PoseStamped, queue_size=1)
        sensor_values_pub = rospy.Publisher('/magnet_sensor_values', Float64MultiArray, queue_size=1)
    time.sleep(1)
    print(W_H_EE)
    print("DET_H_HAND:", DET_H_HAND)
    # Determine the number of sensors
    num_sensors = mag.determine_num_sensors()
    print(f"{num_sensors} sensors received")
    
    # Perform calibration or retrieve old calibration data
    calib_data = mag.calib_flow(num_sensors,folder_path)

    # Ask if user wants to show the quiver plot    
    show_plot = mag.answer("Do you want to plot 3d model with predicted(least square) magnetic position? (y/n): ")
    
    # Initialize arrays for storing data
    rawdata_to_csv = np.empty(num_sensors * 3, dtype=float)
    cleandata_to_csv = np.empty(num_sensors * 3, dtype=float)
    poses_to_csv = np.empty(7, dtype=float)

    # Compute the averaged north vector for 15 seconds
    nord_av = process_nord_data(mag, calib_data)
    print(f"Averaged nord: {nord_av}")

    # If robot is connected, initialize additional data structures
    if with_robot:
        
        # Colors for point cloud in RViz
        red = np.array([255, 255, 255, 20])
        green = np.array([120, 40, 255, 20])
        purple = np.array([80, 60, 180, 20])
        boh = np.array([40, 100, 210, 50])
        
        # Compute the pose for the home position

        
        # Compute homogeneous transformation from pose to world frame
        
        
        # Transform north vector from sensor to awl frame
        awl_nord_av = np.dot(HAND_R_DET, nord_av)

        # Transform north vector from awl to world frame
        world_nord_av = np.dot((W_H_EE[:3, :3]), awl_nord_av)
        print(f"Pure rotation nord: {np.average(world_nord_av, axis=1)}")
        toshow_fixed_nord = np.squeeze(np.transpose(np.average(world_nord_av, axis=1))/200)
        
        print(np.rad2deg(np.arctan2(world_nord_av[1], world_nord_av[0])))
        print(f"Averaged world nord: {toshow_fixed_nord}")

    
    # Ask if user wants to start receiving data
    should_continue = mag.answer("Start receiving data? (y/n)")
    
    if not should_continue:
        mag.flush_input_buffer()
        sys.exit()
    
    # Initial guess for nonlinear least square parameters [theta, phi, rx, ry, rz]
    initial_r = np.array([0, 0, 0.5])
    initial_theta = np.pi/2
    initial_phi = np.pi/2
    default_params = np.hstack((initial_r, initial_theta, initial_phi))
    initial_params = default_params
    
    # Set up 3D plot if requested
    if show_plot:
        vis, points = mag.setup_3d_plot(MESH_PATH)
        ctr = vis.get_view_control()
        ctr.set_zoom(1.5)
        ctr.set_front([0, 0, 1])
        ctr.set_up([-1, 0, 0])
        mag.update_3d_versor(vis, points, initial_r, mag.spherical_to_cartesian(initial_theta, initial_phi), False)
    
    # Initialize variables for data processing

    time_averaged_stats = 10
    window_results_size = 8
    window_results = np.ones((window_results_size, 5))
    window_results_index = 0

    
    magnet_found = False
    mag.flush_input_buffer()
    
    cycle_time = time.time()
    cycle_count = 1
    av_std = np.zeros(1)
    av_distance = np.zeros(1)
    av_rsquared = np.zeros(1)
    av_neval = np.zeros(1)
   
    try:
        while True and not rospy.is_shutdown():
           
            data_line = mag.receive_sensor_data_line(calib_data)
            if data_line is None:
                continue
            
            to_LS_optimizer = data_line - nord_av

            
            if with_robot:

                # Assign the instantaneous ee pose

                W_H_actpose = W_H_EE
                
                # Transform sensed magnetic field from sensor to awl frame
                awl_sensed_line = np.dot(HAND_R_DET, data_line)

                # Transform sensed magnetic field from awl to world frame
                world_sensed_line = np.dot((W_H_actpose[:3, :3]), awl_sensed_line)
                
                # Subtract the sensed north to reject it
                annihilated_nord = world_sensed_line - world_nord_av
                MAGNET_POS_HAND = np.dot(np.linalg.inv(W_H_actpose), np.transpose(np.hstack((MAGNET_POS_W,1))))
                
                annnord_awl = np.dot(np.linalg.inv(W_H_actpose[:3, :3]), annihilated_nord)
                hand_sensed_line = np.dot(np.linalg.inv(W_H_actpose[:3, :3]), world_sensed_line)
                det_sensed_line = np.dot(DET_R_HAND, hand_sensed_line)
                annord_det = np.dot(DET_R_HAND, annnord_awl)
                to_LS_optimizer = annord_det
                
                # Scale and transpose data to later show on RViz graphically
                toshow_nord_diff = np.squeeze(np.transpose(np.average(annihilated_nord, axis=1))/200)
                toshow_sensed_line = np.squeeze(np.transpose(np.average(world_sensed_line, axis=1))/200)
                
                # Append data to numpy matrices for later saving
                cleandata_to_csv = np.vstack((cleandata_to_csv, np.hstack((np.transpose(annihilated_nord)))))
                # poses_to_csv = np.vstack((poses_to_csv, pose))
            
            # print(to_LS_optimizer)
            B_measured = np.reshape(np.transpose(to_LS_optimizer), (3*num_sensors))
            B_measured_with_nord = np.reshape(np.transpose(det_sensed_line), (3*num_sensors))
            norm_from_nord = np.linalg.norm((to_LS_optimizer), axis=0)
            av_normnord = np.average(norm_from_nord)
            start_ls_time = time.time()
            result = least_squares(mag.mag_residual, initial_params, max_nfev=20, args=(sensor_offsets, B_measured))
            ls_time = time.time() - start_ls_time
            #lost_ls_ms = (time.time() - crono_robot_comp)
            initial_params = result.x
            
            

                # Calculate and print R-squared
            predicted_B = mag.mag_residual(result.x, sensor_offsets, B_measured) + B_measured
            r_squared = r2_score(B_measured,predicted_B)
            av_rsquared = np.vstack((av_rsquared, r_squared))
            av_neval = np.vstack((av_neval, result.nfev))
            

            # Initialize empty vector for ls results
            r_opt = None
            m_opt = None
            magnet_found = False

            # print(f"B_measured : {B_measured}")
            # print(f"B_measured_with_nord : {B_measured_with_nord}")

            #C_measured = np.reshape((B_measured, B_measured_with_nord, ls_time, r_squared), (1,26))
            #C_measured = np.concatenate((B_measured.reshape(1,12), B_measured_with_nord.reshape(1,12), [ls_time, r_squared].reshape(1,2))).reshape(1, 26)
            
            C_measured = np.zeros((1,26))  # Initialize empty array of correct size
            index = 0

            for i in range(12):
                C_measured[0,index] = B_measured[i]
                index += 1

            for i in range(12):
                C_measured[0,index] = B_measured_with_nord[i]
                index += 1

            # Add ls_time
            C_measured[0,index] = ls_time
            index += 1

            # Add r_squared
            C_measured[0,index] = r_squared

            print(C_measured)
            msg_mag_sensor_values = utils.prepare_multiarray_message_for_python(C_measured)
            sensor_values_pub.publish(msg_mag_sensor_values)

            pos_mag_ros_data_hand = np.ones((3))*(-1)

            # If the prediction is good:
            if r_squared > high_R2:
                
                magnet_found  = True
                mag_x, mag_y, mag_z, theta, phi = result.x
                window_results[window_results_index] = mag_x, mag_y, mag_z, theta, phi
                window_results_index = (window_results_index + 1) % window_results_size
                
                filtered_results = np.average(window_results, axis=0)
                std_results = np.round(np.average(np.std(window_results[:, :3], axis=0)), 4)
                
                r_opt = filtered_results[:3]
                m_opt = mag.spherical_to_cartesian(filtered_results[3], filtered_results[4])
                
                pred_distance = np.round((np.linalg.norm(r_opt)), 4)
                av_std = np.vstack((av_std, std_results))
                av_distance = np.vstack((av_distance, pred_distance))


                r_opt_to_send = np.hstack((r_opt,1))
                m_opt_to_send = np.hstack((m_opt,1))
                pos_mag_ros_data_hand = np.dot(HAND_H_DET, np.transpose(r_opt_to_send))
                orient_mag_ros_data_hand = np.dot(HAND_H_DET, np.transpose(m_opt_to_send))
                
                mag_pos_msg = PoseStamped()
                mag_pos_msg.header.stamp = rospy.Time.now()
                mag_pos_msg.header.frame_id = "world"
                mag_pos_msg.pose.position.x = pos_mag_ros_data_hand[0]
                mag_pos_msg.pose.position.y = pos_mag_ros_data_hand[1]
                mag_pos_msg.pose.position.z = pos_mag_ros_data_hand[2]
                mag_pos_msg.pose.orientation.x = orient_mag_ros_data_hand[0]
                mag_pos_msg.pose.orientation.y = orient_mag_ros_data_hand[1]
                mag_pos_msg.pose.orientation.z = orient_mag_ros_data_hand[2]


            # If the prediction is terrible, reset paramenters
            elif r_squared < low_R2:
                mag_pos_msg = PoseStamped()
                mag_pos_msg.header.stamp = rospy.Time.now()
                mag_pos_msg.header.frame_id = "world"
                mag_pos_msg.pose.position.x = -1
                mag_pos_msg.pose.position.y = -1
                mag_pos_msg.pose.position.z = -1


                # You may want to add orientation information if available
              
                initial_params = default_params

            #If the prediction is not good but neither terrible, search starting from the last parameter but do not public the data
            elif r_squared > low_R2 and r_squared < high_R2:
                mag_pos_msg = PoseStamped()
                mag_pos_msg.header.stamp = rospy.Time.now()
                mag_pos_msg.header.frame_id = "world"
                mag_pos_msg.pose.position.x = -1
                mag_pos_msg.pose.position.y = -1
                mag_pos_msg.pose.position.z = -1

        
            if args.debug:
                print(f"Debug: Magnet position - x: {mag_pos_msg.pose.position.x:.4f}, y: {mag_pos_msg.pose.position.y:.4f}, z: {mag_pos_msg.pose.position.z:.4f}")
            else:
                mag_pos_pub.publish(mag_pos_msg)
            # Update 3D plot if enabled
            if show_plot:
                mag.update_3d_versor(vis, points, r_opt, m_opt, magnet_found)
            
            rawdata_to_csv = np.vstack((rawdata_to_csv, np.hstack((np.transpose(data_line)))))
            print(f"Magnet real pos detector: {MAGNET_POS_HAND}")
            print(f"Magnet estimated pos detector: {pos_mag_ros_data_hand}")
            # Publish point cloud data for visualization if robot is connected
            if with_robot:
                utils.pcl2_pub_from_list("data1", [((toshow_fixed_nord) + [0.2, 0, 0.2]), [0.2, 0, 0.2]], red)
                utils.pcl2_pub_from_list("data2", [((toshow_sensed_line) + [0.2, 0, 0.2]), [0.2, 0, 0.2]], purple)
                utils.pcl2_pub_from_list("data3", [((world_nord_av[:, 0]/200) + [0.2, 0, 0.2]), ((world_nord_av[:, 1]/200) + [0.2, 0, 0.2]),
                                                ((world_nord_av[:, 2]/200) + [0.2, 0, 0.2]), ((world_nord_av[:, 3]/200) + [0.2, 0, 0.2]), [0.2, 0, 0.2]], green)
                utils.pcl2_pub_from_list("data4", [((toshow_nord_diff) + [0.2, 0, 0.2]), [0.2, 0, 0.2]], boh)
                
                if magnet_found:
                    # Riferito al detector

                   
                    north_point, south_point = mag.mag_point_cloud(r_opt, m_opt)
                    
                    paral_opt = np.vstack((north_point, south_point))

                    paral_opt = np.transpose(np.hstack((paral_opt, np.ones((paral_opt.shape[0], 1)))))
                    ls_magnet_awl = np.dot(HAND_H_DET, paral_opt)
                    ls_magnet_world = np.dot(W_H_actpose, ls_magnet_awl)
                    utils.pcl2_pub_from_list("magnet", [ls_magnet_world[:3, 0], ls_magnet_world[:3, 1]], red)



                    

            
            # Print statistics every time_averaged_stats second
            if time.time() - cycle_time > time_averaged_stats:
                hz = 1 / ((time.time() - cycle_time) / cycle_count)
                cycle_time = time.time()
                cycle_count = 0
                print(to_LS_optimizer)
                print("-------------- STATS AVERAGED EVERY", time_averaged_stats, "SECONDS--------------")
                print(f"hz: {hz}")
                print(f"Norm from nord: {np.round(av_normnord,4)}")
                print(f"Num of evaluation: {np.average(av_neval[1:])}")
                print(f"R squared of prediction: {np.average(av_rsquared[1:])}")
                if magnet_found:
                   
                    print(f"Distance from det: {np.round(np.average(av_distance[1:]),4)}")
                    print(f"av. STD of sliding window: {np.round(np.average(av_std[1:]),4)}")
                    print(f"STD of averaged pos: {np.round(np.std(av_distance[1:]),4)}")                                        
                                     
                else:
                    print("Magnet not found")
                av_std = np.zeros(1)
                av_distance = np.zeros(1)
                av_rsquared = np.zeros(1)
                av_neval = np.zeros(1)
            
            cycle_count += 1
    except KeyboardInterrupt:

        pass
    except Exception as e:
        print("Exception in main loop: ", e)
        pass

    except rospy.ROSInterruptException:
        pass
    finally:
        if show_plot:
            vis.destroy_window()
            
        try:
            mag.end_mag_connection()            
        except Exception as e:
            print(f"Error ending magnetic connection: {e}")

def save_data(folder_path, rawdata_to_csv, nord_av, poses_to_csv, cleandata_to_csv, with_robot):
    current_datetime = datetime.now().strftime("%m_%d_%H_%M_%S")
    filenames = [
        os.path.join(folder_path, f"{BASE_FILENAME}_{current_datetime}.csv"),
        os.path.join(folder_path, f"{BASE_FILENAME_NORDAV}_{current_datetime}.csv"),
        os.path.join(folder_path, f"{BASE_FILENAME_POSE}_{current_datetime}.csv"),
        os.path.join(folder_path, f"{BASE_FILENAME_VALUES}_{current_datetime}.csv"),
        os.path.join(folder_path, f"{BASE_FILENAME_HOM}_{current_datetime}.csv"),
        os.path.join(folder_path, f"{BASE_FILENAME_EEMAG_POS}_{current_datetime}.csv")
    ]

    try:
        print("Saving raw sensor data...")
        np.savetxt(filenames[0], rawdata_to_csv, delimiter=',', fmt='%0.7f')
        
        if with_robot:
            print("Saving nord average data...")
            np.savetxt(filenames[1], nord_av, delimiter=',', fmt='%0.7f')
            
            print("Saving pose data...")
            np.savetxt(filenames[2], poses_to_csv, delimiter=',', fmt='%0.7f')
            
            print("Saving clean sensor data...")
            np.savetxt(filenames[3], cleandata_to_csv, delimiter=',', fmt='%0.7f')

        print("Files saved successfully.")
    except Exception as e:
        print(f"An error occurred while saving data: {e}")

    print("Closing...")
def waypoint_stop_path(fr3, waypoint):

    plan, _, _, _ = fr3.plan_to_a_waypoint(waypoint, 5.0, 50, 0.15, 0.05)
    # fr3.display_trajectory(plan)
    fr3.execute_plan(plan,False, True)



if __name__ == '__main__':
    main()