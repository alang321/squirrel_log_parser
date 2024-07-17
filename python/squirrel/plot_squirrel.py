import numpy as np
from matplotlib import pyplot as plt
import os
from pathlib import Path

from tools.parselog import parselog
from tools.plotting_tools import *


@seaborn_style()
def pretty_plot(filename):
    ac_data = get_ac_data(filename)
    plt.figure()
    plot_commands(ac_data)

def trim_to_timerange(timestamps, data, t0, t1):
    start_idx = np.searchsorted(timestamps, t0)
    end_idx = np.searchsorted(timestamps, t1)
    timestamps_trimmed = timestamps[start_idx:end_idx]
    data_trimmed = data[start_idx:end_idx]
    return timestamps_trimmed, data_trimmed 
    

if __name__ == "__main__":
    folder = 'flight5' # change only this when reading a new file

    dir = "./logs/" + folder + "/"
    file = Path(os.listdir(dir)[0]).stem + ".log"
    print((dir[2:] + file))
    log_data = parselog((dir[2:] + file))

    ac_data = log_data["aircrafts"][0]["data"]
    ac_msgs = log_data["msgs"]
    print(ac_data.keys())  # check all logged messages

    # Check sub-fields of certain messages
    print(f"gyro: {ac_data['IMU_GYRO'].keys()}")
    print(f"accel: {ac_data['IMU_ACCEL'].keys()}")
    print(f"gps: {ac_data['DESIRED'].keys()}")

    # ==== Plot with functions from spin_plot_tools ====
    # Plot log overviews
    #plot_overview(filename)
    #plot_ins_overview(filename)
    # plot_energy_overview(filename)

    # Single plot

    # ========== Plot manually ==============

    t0 = 20
    t1 = 50

    #mode_t = ac_data["PPRZ_MODE"]["timestamp"]
    #mode = ac_data["PPRZ_MODE"]['ap_mode']

    #cmd_t = ac_data["COMMANDS"]["timestamp"]
    #cmd = ac_data["COMMANDS"]["values"][:, 1]

    att_t = ac_data["ATTITUDE"]["timestamp"]
    phi = np.rad2deg(ac_data["ATTITUDE"]["phi"])
    theta = np.rad2deg(ac_data["ATTITUDE"]["theta"])

    att_d_t = ac_data["DESIRED"]["timestamp"]
    phi_d = ac_data["DESIRED"]["roll_alt"]
    theta_d = ac_data["DESIRED"]["pitch_alt"]

    imu_gyro_data_timestamp = ac_data['IMU_GYRO']['timestamp']
    imu_gyro_data_p = ac_data['IMU_GYRO']['gp_alt']
    imu_gyro_data_q = ac_data['IMU_GYRO']['gq_alt']
    imu_gyro_data_r = ac_data['IMU_GYRO']['gr_alt']

    imu_accel_data_timestamp = ac_data['IMU_ACCEL']['timestamp']
    imu_accel_data_ax = ac_data['IMU_ACCEL']['ax']
    imu_accel_data_ay = ac_data['IMU_ACCEL']['ay']
    imu_accel_data_az = ac_data['IMU_ACCEL']['az']

    servo_t = ac_data['SERIAL_ACT_T4_IN']['timestamp']
    servo_1_load_percent = ac_data['SERIAL_ACT_T4_IN']['servo_1_load_int'] / 10
    servo_2_load_percent = ac_data['SERIAL_ACT_T4_IN']['servo_2_load_int'] / 10
    servo_3_load_percent = ac_data['SERIAL_ACT_T4_IN']['servo_3_load_int'] / 10
    servo_4_load_percent = ac_data['SERIAL_ACT_T4_IN']['servo_4_load_int'] / 10
    servo_5_load_percent = ac_data['SERIAL_ACT_T4_IN']['servo_5_load_int'] / 10
    servo_1_ang = ac_data['SERIAL_ACT_T4_IN']['servo1_angle_alt']
    servo_2_ang = ac_data['SERIAL_ACT_T4_IN']['servo2_angle_alt']
    servo_3_ang = ac_data['SERIAL_ACT_T4_IN']['servo3_angle_alt']
    servo_4_ang = ac_data['SERIAL_ACT_T4_IN']['servo4_angle_alt']
    servo_5_ang = ac_data['SERIAL_ACT_T4_IN']['servo5_angle_alt']

    servo_cmd_t = ac_data['SERIAL_ACT_T4_OUT']['timestamp']
    servo_1_cmd_ang_d = ac_data['SERIAL_ACT_T4_OUT']['servo1_angle_alt']
    servo_2_cmd_ang_d = ac_data['SERIAL_ACT_T4_OUT']['servo2_angle_alt']
    servo_3_cmd_ang_d = ac_data['SERIAL_ACT_T4_OUT']['servo3_angle_alt']
    servo_4_cmd_ang_d = ac_data['SERIAL_ACT_T4_OUT']['servo4_angle_alt']
    servo_5_cmd_ang_d = ac_data['SERIAL_ACT_T4_OUT']['servo5_angle_alt']

    fig, ax = plt.subplots(4,2)
    fig.tight_layout() # Or equivalently,  "plt.tight_layout()"


    # Unpack message fields directly
    ax[0, 0].set_title("Gyro")
    ax[0, 0].plot(imu_gyro_data_timestamp, imu_gyro_data_p, label='p')
    ax[0, 0].plot(imu_gyro_data_timestamp, imu_gyro_data_q, label='q')
    ax[0, 0].plot(imu_gyro_data_timestamp, imu_gyro_data_r, label='r')
    ax[0, 0].set_xlabel("Time [s]")
    ax[0, 0].set_ylabel("deg/s [-]")
    ax[0, 0].legend()

    # Unpack message fields directly
    ax[1, 0].set_title("Accel")
    ax[1, 0].plot(imu_accel_data_timestamp, imu_accel_data_ax, label='x')
    ax[1, 0].plot(imu_accel_data_timestamp, imu_accel_data_ay, label='y')
    ax[1, 0].plot(imu_accel_data_timestamp, imu_accel_data_az, label='z')
    ax[1, 0].set_xlabel("Time [s]")
    ax[1, 0].set_ylabel("m/s2")
    ax[1, 0].legend()

    # Unpack message fields directly
    ax[2, 0].set_title("Servo1 Angle Tracking")
    ax[2, 0].plot(servo_t, servo_1_ang, label='servo1')
    ax[2, 0].plot(servo_cmd_t, servo_1_cmd_ang_d, label='servo1 desired')
    ax[2, 0].set_xlabel("Time [s]")
    ax[2, 0].set_ylabel("Angle [deg]")
    ax[2, 0].legend()

    # Unpack message fields directly
    ax[3, 0].set_title("Roll")
    ax[3, 0].plot(att_t, phi, label='roll')
    ax[3, 0].plot(att_d_t, phi_d, label='roll desired')
    ax[3, 0].set_xlabel("Time [s]")
    ax[3, 0].set_ylabel("Roll [deg]")
    ax[3, 0].legend()

    ax[0, 1].set_title("Servo load")
    ax[0, 1].plot(servo_t, servo_1_load_percent, label='servo1')
    ax[0, 1].plot(servo_t, servo_2_load_percent, label='servo2')
    ax[0, 1].set_xlabel("Time [s]")
    ax[0, 1].set_ylabel("load [%]")
    ax[0, 1].legend()

    ax[1, 1].set_title("Servo load")
    ax[1, 1].plot(servo_t, servo_3_load_percent, label='servo3')
    ax[1, 1].plot(servo_t, servo_4_load_percent, label='servo4')
    ax[1, 1].set_xlabel("Time [s]")
    ax[1, 1].set_ylabel("load [%]")
    ax[1, 1].legend()

    ax[2, 1].set_title("Servo load")
    ax[2, 1].plot(servo_t, servo_5_load_percent, label='servo5')
    ax[2, 1].set_xlabel("Time [s]")
    ax[2, 1].set_ylabel("load [%]")
    ax[2, 1].legend()


    # Unpack message fields directly
    ax[3, 1].set_title("Pitch")
    ax[3, 1].plot(att_t, theta, label='pitch')
    ax[3, 1].plot(att_d_t, theta_d, label='pitch desired')
    ax[3, 1].set_xlabel("Time [s]")
    ax[3, 1].set_ylabel("Pitch [deg]")
    ax[3, 1].legend()


    plt.show()




