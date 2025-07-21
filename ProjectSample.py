import time
import socket
import math
# No need to modify this function
def udp_sender(server_ip, server_port, message):
    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send the message to the server
        client_socket.sendto(message.encode(), (server_ip, server_port))
        print(f"Command sent to {server_ip}:{server_port}, command: {message}")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the socket
        client_socket.close()

# No need to modify this function
def send_command(server_ip, server_port, angle_list, color_list):
    '''
    this function is used to send the command to the robot. If you are using the simulator, you need to set the robot to Server Mode in the setting page.
    angle_list: list of angles
    color_list: list of colors
    '''
    message = ",".join([str(i) for i in angle_list])
    message += ","
    message += ",".join([str(i) for i in color_list])
    udp_sender(server_ip, server_port, message)

# No need to modify this function
def concatenate_trajectories(traj_list_1, traj_list_2):
    '''
    this function is used to concatenate the trajectories
    traj_list_1: list of list, each list is a trajectory
    traj_list_2: list of list, each list is a trajectory
    '''
    assert len(traj_list_1) == len(traj_list_2)
    traj_list = [[] for _ in range(3)]
    for i in range(len(traj_list_1)):
        traj_list[i] = traj_list_1[i] + traj_list_2[i]
    return traj_list

# No need to modify this function
def trajectory_to_file(traj_list, color_list, prefix="angle"):
    '''
    this function is used to save the trajectory to a file
    traj_list: list of list, each list is a trajectory
    color_list: list of colors
    prefix: prefix of the file name
    '''
    result = prefix + ";"
    for point in traj_list:
        result += ",".join([str(i) for i in point])
        result += ";"
    for color in color_list:
        result += ",".join([str(i) for i in color])
        result += ";"
    f = open("trajectory_joint_space_with_color.txt", mode="w")
    f.write(result)
    f.close()

# Example 1: Linear Trajectory
def linear_trajectory(start_pos, end_pos, end_time,main_traj, frequency=50):
    '''
    this function is used to generate a linear trajectory
    start_pos: list of start position (or angles)
    end_pos: list of end position (or angles)
    end_time: time spend of the trajectory (in seconds)
    frequency: frequency of the trajectory
    '''
    
    # Get the trajectory
    # Define the result
    result = [[], [], []]

    # List of time for each frame
    time_list = []
    for i in range(end_time * frequency):
        time_list.append(i / frequency)

    # Repeat the process for each joint
    for i in range(3):
        # Get the coefficients
        a0 = start_pos[i]
        a1 = (end_pos[i] - start_pos[i]) / end_time

        # Generate the trajectory
        for t in time_list:
            # linear interpolation
            main_traj[i].append(a0 + a1 * t)

    return main_traj

# Task 1: Cubic Trajectory for Two Points
def cubic_trajectory(start_pos, end_pos, start_vel, end_vel, end_time,main_traj, frequency=50):
    '''
    this function is used to generate a cubic trajectory
    start_pos: list of start position 
    end_pos: list of end position 
    start_vel: list of start velocity
    end_vel: list of end velocity
    end_time: time spend of the trajectory (in seconds)
    frequency: frequency of the trajectory
    '''

    # Get the trajectory
    # Define the result
    result = [[], [], []]

    # List of time for each frame
    time_list = []
    for i in range(end_time * frequency):
        time_list.append(i / frequency)

    for i in range(3):
        # Get the coefficients
        a2 = start_vel[i]
        a3 = start_pos[i]
        a0 = (-2*end_pos[i]+2*start_pos[i]+end_time*(start_vel[i]+end_vel[i]))/end_time**3
        a1 = (end_pos[i]-start_pos[i]-end_time*start_vel[i]-(end_time**3)*a0)/end_time**2
        
        # Generate the trajectory
        for t in time_list:
            # cubic interpolation
            main_traj[i].append(a0*t**3+a1*t**2+a2*t+a3)
    return main_traj

# Task 3: Inverse Kinematics
def inverse_kinematics(task_space_pos):
    '''
    this function is used to generate the joint space position from the task space position
    task_space_pos: task space position 
    '''
    
    # This function converts task space to joint space
    # Input: Task space coord. For example TaskSpace2JointSpace(10, 20, 10)
    # Output: A list of joint angles. For example [-26.56505117707799, 43.33991400145487, -65.29185017692065]
    l1 = 7.7
    l2 = 12.7
    l3 = 2.5
    l4 = 11.6
    x = task_space_pos[0]
    y = -task_space_pos[1]
    z = task_space_pos[2]
    # y = -y
    angles = [0 for i in range(3)]
    l23 = math.sqrt(l2 * l2 + l3 * l3)
    alpha = math.atan(l3/l2)

    if x == 0:
        angles[0] = math.pi / 2
    else:
        if x > 0:
            angles[0] = math.atan(-y/x)
        else:
            angles[0] = math.pi - math.atan(y/x)

    A = -y * math.sin(angles[0]) + x * math.cos(angles[0])

    B = z - l1

    tmp = (A * A + B * B - (l23 * l23 + l4 * l4)) / (2 * l23 * l4)
    if tmp < -1:
        tmp = -0.999999
    if tmp > 1:
        tmp = 0.99999
    angles[2] = -math.acos(tmp)
    if (A * (l23 + l4 * math.cos(angles[2])) + B * l4 * math.sin(angles[2])) > 0:
        angles[1] = math.atan((B * (l23 + l4 * math.cos(angles[2])) - A * l4 * math.sin(angles[2])) /
                               (A * (l23 + l4 * math.cos(angles[2])) + B * l4 * math.sin(angles[2])))
    else:
        angles[1] = math.pi - math.atan((B * (l23 + l4 * math.cos(angles[2])) - A * l4 * math.sin(angles[2])) /
                                          -(A * (l23 + l4 * math.cos(angles[2])) + B * l4 * math.sin(angles[2])))

    angles[0] = angles[0] / math.pi * 180 - 90
    angles[1] = (angles[1] + alpha) / math.pi * 180
    angles[2] = (angles[2] - alpha) / math.pi * 180

    return angles

# Example 2: Static Color List
def static_color_list(color, time_spend, main_colour, frequency=50):
    '''
    this function is used to generate a list of static color
    color: list of color
    time_spend: time spend of the trajectory (in seconds)
    frequency: frequency of the trajectory
    '''
    # List of time for each frame
    for j in range(time_spend * frequency):
        for i in range(3):
            main_colour[i].append(color[i])
    return main_colour

# Task 4(Optional): Generate the color trajectory
def generate_linear_color_list(start_color, end_color, end_time, main_colour, frequency=50):
    '''
    this function is used to generate a list color that changes linearly from start_color to end_color
    start_color: list of start color
    end_color: list of end color
    end_time: time spend of the trajectory (in seconds)
    frequency: frequency of the trajectory
    '''
    # Define the result
    result = [[], [], []]

    # Repeat the process for each red, green, blue component
    for i in range(3):
        # Get the coefficients
        a0 = start_color[i]
        a1 = (end_color[i] - start_color[i]) / end_time

        # List of time for each frame
        time_list = []
        for i in range(end_time * frequency):
            time_list.append(i / frequency)

        # Generate the trajectory
        for t in time_list:
            main_colour[i].append(a0+a1*t)
    return main_colour


# Task 4: Generate the trajectories
if __name__ == "__main__":
    # Set the frequency
    frequency = 50
    task_space_trajectory = [[],[],[]]
    color_trajectory = [[],[],[]]

    # Generate the task space trajectory


    task_space_trajectory_1 = linear_trajectory([0, 0, 0], [10,10,10], 10,task_space_trajectory, 1)
    task_space_trajectory_2 = linear_trajectory([0, 0, 0], [10,10,10], 10,task_space_trajectory, 1)


    # generate colour trajectory
    color_trajectory_1 = static_color_list([0, 0, 0], 2,color_trajectory, frequency)
    color_trajectory_2 = static_color_list([255, 255, 255], 2,color_trajectory, frequency)


    # Generate the joint space trajectory
    joint_space_trajectory = [[], [], []]
    for i in range(len(task_space_trajectory[0])):
        joint_space_point = inverse_kinematics([task_space_trajectory[0][i], task_space_trajectory[1][i], task_space_trajectory[2][i]])
        for j in range(3):
            joint_space_trajectory[j].append(joint_space_point[j])

    # Generate the trajectory file for simulator
    trajectory_to_file(joint_space_trajectory, color_trajectory, "angle")

    # No need to modify the following code
    # Send the command to the robot
    # Please make sure the robot is in Server Mode in the setting page of the simulator
    command_length = len(joint_space_trajectory[0])

    server_ip = "localhost" 
    server_port = 5000  # Replace with the server's port
    for i in range(command_length):
        angle_list = [joint_space_trajectory[0][i], joint_space_trajectory[1][i], joint_space_trajectory[2][i]]
        color_list = [color_trajectory[0][i], color_trajectory[1][i], color_trajectory[2][i]]
        send_command(server_ip, server_port, angle_list, color_list)
        time.sleep(1/frequency)
    
    
