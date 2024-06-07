import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from cfusdlog import decode
from tqdm import tqdm
import matplotlib.patches as patches

data_total = []

def plot_avg_std_error(data_list, data_len, directory):

    batch_data_points = len(data_list)
    
    for data in data_list:
        for key, value in data.items():
            if (len(value) < data_len) and (key != 'timestamp'):
                value.extend([value[-1] * (len(value) - data_len)])
    error_x = [np.abs(np.subtract(data['kalman.stateX'], data['ctrlNN.set_x'])) for data in data_list]
    error_y = [np.abs(np.subtract(data['kalman.stateY'], data['ctrlNN.set_y'])) for data in data_list]
    error_z = [np.abs(np.subtract(data['kalman.stateZ'], data['ctrlNN.set_z'])) for data in data_list]

    std_x = np.std(error_x, axis=0)
    std_y = np.std(error_y, axis=0)
    std_z = np.std(error_z, axis=0)

    avg_err_x = [point / batch_data_points for point in sum(error_x)]
    avg_err_y = [point / batch_data_points for point in sum(error_y)]
    avg_err_z = [point / batch_data_points for point in sum(error_z)]

    fig = plt.figure()
    fig, (ax_x, ax_y, ax_z) = plt.subplots(3)

    sample_points = range(data_len)

    ax_x.scatter(sample_points, avg_err_x, marker='o', label='Average X Error', color='red', s=0.1)
    ax_x.fill_between(sample_points, avg_err_x-std_x, avg_err_x+std_x, color='red', alpha=0.4)
    ax_y.scatter(sample_points, avg_err_y,marker='o', label='Average Y Error', color='blue', s=0.1)
    ax_y.fill_between(sample_points, avg_err_y-std_y, avg_err_y+std_y, color='blue', alpha=0.4)    
    ax_z.scatter(sample_points, avg_err_z, marker='o', label='Average Z Error', color='green', s=0.1)
    ax_z.fill_between(sample_points, avg_err_z-std_z, avg_err_z+std_z, color='green', alpha=0.4)

    ax_x.set_yticks(np.arange(0, 0.5, step=0.1))
    ax_y.set_yticks(np.arange(0, 0.5, step=0.1))
    ax_z.set_yticks(np.arange(0, 0.5, step=0.1))

    ax_x.grid()
    ax_y.grid()
    ax_z.grid()

    ax_x.legend()
    ax_y.legend()
    ax_z.legend()

    plt.savefig(directory + "avg_error.jpg", dpi=800) 
    # stateX = sum([data['kalman.stateY'] for data in data_list])
    # stateX = sum([data['kalman.stateX'] for data in data_list])
    # print(stateX)
    # for batch in data:
    #     print(type(batch))

def plot_error(data, directory):
    estimatex = data['kalman.stateX']
    estimatey = data['kalman.stateY']
    estimatez = data['kalman.stateZ']

    desiredx = data['ctrlNN.set_x']
    desiredy = data['ctrlNN.set_y']
    desiredz = data['ctrlNN.set_z']

    timestep = data['timestamp']

    fig = plt.figure() 
    ax = fig.add_subplot(111) 

    ax.scatter(timestep, np.subtract(estimatex, desiredx), s=0.1, label='X Error')
    ax.scatter(timestep, np.subtract(estimatey, desiredy), s=0.1, label='Y Error')
    ax.scatter(timestep, np.subtract(estimatez, desiredz), s=0.1, label='Z Error')

    ax.set_ylim(-1.3, 1.3)

    ax.legend(markerscale=5, loc='upper left')

    # legend = ax.legend(frameon=True)
    # for legend_handle in legend.legendHandles:
    #     legend_handle._legmarker.set_markersize(9)

    # plt.show()
    plt.tight_layout()
    plt.savefig(directory + "error.jpg", dpi=800) 

def plot_traj(data, directory):
    flat = False
    draw_obst = True
    # # Extracting individual columns for trajectory
    # viconx = np.array(data['locSrv.x'])
    # vicony = np.array(data['locSrv.y'])
    # viconz = np.array(data['locSrv.z'])

    # # Extracting columns for orientation (quaternions)
    # quat_columns = ['locSrv.qx', 'locSrv.qy', 'locSrv.qz', 'locSrv.qw']

    estimatex = data['stateEstimate.x']
    estimatey = data['stateEstimate.y']
    estimatez = data['stateEstimate.z']

    desiredx = data['ctrlNN.set_x']
    desiredy = data['ctrlNN.set_y']
    # desiredz = data['ctrlNN.set_z']

    timestep = data['timestamp']

    # Extracting columns for orientation (quaternions)
    # quat_columns = ['locSrv.qx', 'locSrv.qy', 'locSrv.qz', 'locSrv.qw']
    # quaternions = data[quat_columns].to_numpy()

    # Create a 3D figure
    if flat:
        fig, (ax_x, ax_y, ax_z) = plt.subplots(3)
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111)



    # Plotting the trajectory
    if flat:
        ax_x.scatter(timestep, estimatex, marker='o', label='Trajectory Flown', color='blue', s=0.1)
        ax_y.scatter(timestep, estimatey, marker='o', label='Trajectory Flown', color='blue', s=0.1)  
        ax_z.scatter(timestep, estimatez, marker='o', label='Trajectory Flown', color='blue', s=0.1)

        ax_x.scatter(timestep, desiredx, marker='o', label='Trajectory Flown', color='red', s=0.1)  
        ax_y.scatter(timestep, desiredy, marker='o', label='Trajectory Flown', color='red', s=0.1)  
        ax_z.scatter(timestep, desiredz, marker='o', label='Trajectory Flown', color='red', s=0.1)     

        ax_x.set_ylim(-1, 1)
        ax_y.set_ylim(-1, 1)
        ax_z.set_ylim(0, 2) 
    else:  
        ax.scatter(estimatex, estimatey, marker='o', label='Trajectory Flown', color='blue', s=0.1)
        ax.plot(desiredx, desiredy, label='Desired Trajectory', color='red')   
        ax.set(xlim=(-0.5, 5), ylim=(-1, 1))

        # ax.view_init(elev=10, azim=45, roll=0)
        # Set labels
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        # ax.set_xlim(-1, 1)
        # ax.set_ylim(-8, 1)

        # if draw_obst:
        #     rect = patches.Rectangle((-0.2, -3), 0.3, 0.3, linewidth=1, edgecolor='r', facecolor='none')
        # ax.set_zlabel('Z-axis')
        # ax.add_patch(rect)
        ax.legend()
    # ax.plot(viconx, vicony, viconz, marker='o', label='Vicon', color='green', markersize=0.05)

    # # Plotting orientation using three axes
    # for i in range(len(quaternions)):
    #     quat = quaternions[i]
    #     rot = R.from_quat(quat)
    #     axes = np.eye(3)  # X, Y, Z axes in a 3x3 matrix
    #     rotated_axes = rot.apply(axes)
    #     origin = [x[i], y[i], z[i]]

    #     for j, color in enumerate(['red', 'green', 'blue']):
    #         ax.quiver(origin[0], origin[1], origin[2],
    #                   rotated_axes[j, 0], rotated_axes[j, 1], rotated_axes[j, 2],
    #                   length=0.1, color=color, arrow_length_ratio=0.3, label='Orientation' if i == 0 else '')

    # Labeling start and end points of the trajectory
    # ax.text(x[0], y[0], z[0], 'Start', color='green')
    # ax.text(x[-1], y[-1], z[-1], 'End', color='blue')

    # Show legend

    # Show plot
    # plt.show()
    # plt.tight_layout()
    plt.savefig(directory + "trajectory.jpg", dpi=800) 
    # plt.show()

def quaternion_difference(q1, q2):
    # Calculate the quaternion difference (error) between q1 and q2
    # Conjugate of q2 * q1 gives the relative rotation from q1 to q2
    q2_conj = np.array([q2[0], -q2[1], -q2[2], -q2[3]])
    relative_rotation = quaternion_multiply(q2_conj, q1)
    # Convert to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(relative_rotation)
    return rotation_matrix

def quaternion_multiply(q1, q2):
    # Quaternion multiplication
    result = np.zeros(4)
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return result

def quaternion_to_rotation_matrix(q):
    # Convert a quaternion to a rotation matrix
    qw, qx, qy, qz = q
    rotation_matrix = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])
    return rotation_matrix

def calc_error(data):
    # Extracting position elements from DataFrame columns
    position_cols = ['kalman.stateX', 'kalman.stateY', 'kalman.stateZ', 'locSrv.x', 'locSrv.y', 'locSrv.z']

    # Calculate differences to obtain velocity and acceleration
    velocity = data[position_cols].diff().fillna(0)
    acceleration = velocity[position_cols].diff().fillna(0)

    # Extracting angular velocity elements from DataFrame columns
    quat_cols = ['kalman.q0', 'kalman.q1', 'kalman.q2', 'kalman.q3', 'locSrv.qw', 'locSrv.qx', 'locSrv.qy', 'locSrv.qz']

    # Calculate differences for quaternions to get angular velocity
    quat_diff = data[quat_cols].diff().fillna(0)

    # Extract quaternion columns
    kalman_columns = ['kalman.q0', 'kalman.q1', 'kalman.q2', 'kalman.q3']
    locSrv_columns = ['locSrv.qw', 'locSrv.qx', 'locSrv.qy', 'locSrv.qz']
    
    rot_errors = []
    angular_rot_errors = []

    for index, row in data.iterrows():
        # Calculate error for rotation
        kalman_quaternion = row[kalman_columns].values
        locsrv_quaternion = row[locSrv_columns].values
        
        # Calculate error between kalman and locSrv quaternions
        error = quaternion_difference(kalman_quaternion, locsrv_quaternion)
        rot_errors.append(error)

    for index, row in quat_diff.iterrows():
        # Calculate error for rotation
        kalman_quaternion = row[kalman_columns].values
        locsrv_quaternion = row[locSrv_columns].values
        
        # Calculate error between kalman and locSrv quaternion velocity
        error = quaternion_difference(kalman_quaternion, locsrv_quaternion)
        angular_rot_errors.append(error)

    angular_rot_errors = np.array(rot_errors)

    # Calculate average error and standard deviation
    mean_rot_error = np.mean(rot_errors, axis=0)
    std_rot_error = np.std(rot_errors, axis=0)

    mean_angular_rot_error = np.mean(angular_rot_errors, axis=0)
    std_angular_rot_error = np.std(angular_rot_errors, axis=0)

    # Calculate differences for kalman.state and locSrv (to get error in position and velocity)
    # state_diff = data[['kalman.stateX', 'kalman.stateY', 'kalman.stateZ', 'locSrv.x', 'locSrv.y', 'locSrv.z']].diff().fillna(0)
    error_position = np.array([data['kalman.stateX'] - data['locSrv.x'], data['kalman.stateY'] - data['locSrv.y'], data['kalman.stateZ'] - data['locSrv.z']])
    error_velocity = np.array([velocity['kalman.stateX'] - velocity['locSrv.x'], velocity['kalman.stateY'] - velocity['locSrv.y'], velocity['kalman.stateZ'] - velocity['locSrv.z']])
    error_acceleration = np.array([acceleration['kalman.stateX'] - acceleration['locSrv.x'], acceleration['kalman.stateY'] - acceleration['locSrv.y'], acceleration['kalman.stateZ'] - acceleration['locSrv.z']])

    # Calculate mean and standard deviation for error in position and velocity
    error_position_mean = np.mean(error_position, axis=1)
    error_position_std_dev = np.std(error_position, axis=1)

    error_velocity_mean = np.mean(error_velocity, axis=1)
    error_velocity_std_dev = np.std(error_velocity, axis=1)

    error_acceleration_mean = np.mean(error_acceleration, axis=1)
    error_acceleration_std_dev = np.std(error_acceleration, axis=1)

    with open("localization_statistics.txt", "w") as f:
        # Display mean and standard deviation for error in position and velocity
        print("Mean of error in position:",file=f)
        print(error_position_mean,file=f)
        print("\nStandard Deviation of error in position:",file=f)
        print(error_position_std_dev,file=f)

        print("\nMean of error in velocity:",file=f)
        print(error_velocity_mean,file=f)
        print("\nStandard Deviation of error in velocity:",file=f)
        print(error_velocity_std_dev,file=f)

        print("\nMean of error in Acceleration:",file=f)
        print(error_acceleration_mean,file=f)
        print("\nStandard Deviation of error in velocity:",file=f)
        print(error_acceleration_std_dev,file=f)

        print("\nMean of error in Rotation Matrix:",file=f)
        print(mean_rot_error,file=f)
        print("\nStandard Deviation of error in Rotation Matrix:",file=f)
        print(std_rot_error,file=f)

        print("\nMean of error in Rotation Velocity Matrix:",file=f)
        print(mean_angular_rot_error,file=f)
        print("\nStandard Deviation of error in Rotation Velocity Matrix:",file=f)
        print(std_angular_rot_error,file=f)


def main():
    data_len = 0
    # for folder in tqdm(['domain_randomization_on/0000', 'simstep4_4enc_empty/0000', 'simstep4_4enc_empty/obstacle', 'simstep4_thrust_empty/0000', 'simstep4_4enc_05/0000']):
    for folder in ['random']:
        for filename in os.listdir(folder):
            if ('.' not in filename):
                filename = os.path.join(folder,filename)
                print(f'Evaluating on data {filename}...')
                # print(filename)
                data = decode(filename)["fixedFrequency"]
                if (len(data['stateEstimate.x']) > data_len):
                    data_len = len(data['stateEstimate.x'])
                data_total.append(data)
                # print(data)

                # plot_error(data, filename)
                plot_traj(data, filename)
        # plot_avg_std_error(data_total, data_len, folder)

if __name__ == "__main__":
    main()
