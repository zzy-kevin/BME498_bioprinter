import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import numpy as np
import math
import tkinter.font as tkFont
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from PIL import Image, ImageTk

from segmentation import *
from ui import *
from capture import *


def plot_3d_height_map(height_map):
    # Create figure and 3D axis
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Create coordinate grids
    x = np.arange(0, height_map.shape[1])
    y = np.arange(0, height_map.shape[0])
    X, Y = np.meshgrid(x, y)

    # Create copy of height map and replace zeros with NaN
    masked_height = height_map.copy().astype(float)
    masked_height[masked_height == 0] = np.nan
    
    # Create the depth map (negative of height map)
    depth_map = -masked_height

    # Plot the surface
    surf = ax.plot_surface(X, Y, depth_map, cmap='jet',
                           linewidth=0, antialiased=True)

    # Customize plot
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Height')
    ax.set_title('3D Height Map Visualization')

    # Add color bar
    fig.colorbar(surf, shrink=0.5, aspect=5)
    s = time.gmtime(time.time())
    name = time.strftime("%Y-%m-%d %H:%M:%S", s)
    plt.savefig(str(name).replace(":", "_") + ".jpg")
    plt.clf()

"""
We have a global arm_a and arm_b deg so we can store the current position of the robotic arm.
Note that this is always round to deg per step
We have a global arm_a and arm_b err so we can compensate for movement smaller than our step
This error is considered and updated everytime the arm is moved.
"""
arm_a_deg = 90
arm_b_deg = 90
arm_z = 0.0
dist_per_pix = 5.0
arm_a_err = 0.0
arm_b_err = 0.0
test_times = 0
x_comp = 5
y_comp = -10
cam_pos = 78    #camera position
last_dist = 0


class RoboticArmController:
    def __init__(self, serial_connection):
        self.serial_connection = serial_connection
        self.steps_per_revolution = 200  # Example: 200 steps per motor revolution (1.8° per step)
        self.gear_ratio = 4*4  # Reduction ratio (motor:arm = 1:4) and we are doing quarter microstepping so 4*4
        self.steps_per_deg = self.steps_per_revolution * self.gear_ratio / 360
        self.deg_per_step = 1 / self.steps_per_deg
        self.z_mm_per_step = 0.02  # (4 mm per rot/200 step per rot), no microstepping for z
        print(self.deg_per_step)
        print(self.steps_per_deg)


    def rotate_arm_a(self, degrees, wait):
        """
        Rotates the first arm (arm_a) by a certain degree.
        Args:
            degrees (float): The angle to rotate in degrees (positive for CW, negative for CCW).
            wait (int): The delay between PWM signals in microseconds.
        """
        global arm_a_err
        direction = 0 if degrees > 0 else 1  # CW = 0, CCW = 1, REMEMBER THAT motor is upside down so this is inverted
        compensated_degree = np.fix((degrees + arm_a_err) / self.deg_per_step) * self.deg_per_step
        arm_a_err = compensated_degree - (degrees + arm_a_err)

        steps = abs(compensated_degree) * self.steps_per_revolution * self.gear_ratio / 360
        message = f"rot_a {direction} {int(steps)} {wait}"
        self.serial_connection.write((message + '\n').encode())

        return np.sign(degrees) * compensated_degree

    def rotate_arm_b(self, degrees, wait):
        """
        Rotates the second arm (arm_b) by a certain degree.
        Args:
            degrees (float): The angle to rotate in degrees (positive for CW, negative for CCW).
            wait (int): The delay between PWM signals in microseconds.
        """
        global arm_b_err
        direction = 0 if degrees > 0 else 1  # CW = 0, CCW = 1
        compensated_degree = np.fix((degrees + arm_b_err) / self.deg_per_step) * self.deg_per_step
        arm_b_err = compensated_degree - (degrees + arm_b_err)

        steps = abs(compensated_degree) * self.steps_per_revolution * self.gear_ratio / 360
        message = f"rot_b {direction} {int(steps)} {wait}"
        self.serial_connection.write((message + '\n').encode())

        return np.sign(degrees) * compensated_degree

    def move_arm_linear(self, start_angle_a, start_angle_b, target_x, target_y, l_a, l_b):
        """
        Breakdown the path into small segment and call move_arm to traverse each to obtain a linear path
        """
        global arm_a_deg, arm_b_deg
        x_start = l_a * math.cos(math.radians(start_angle_a)) + \
                  l_b * math.cos(math.radians(start_angle_a + start_angle_b))
        y_start = l_a * math.sin(math.radians(start_angle_a)) + \
                  l_b * math.sin(math.radians(start_angle_a + start_angle_b))
        x_len = target_x - x_start
        y_len = target_y - y_start
        curr_angle_a = start_angle_a
        curr_angle_b = start_angle_b
        break_len = 10
        seg_count = round(max(x_len, y_len) / break_len)
        dy = y_len / seg_count
        dx = x_len / seg_count
        for i in range(seg_count):
            segment_x = x_start + dx*(i+1)
            segment_y = y_start + dy*(i+1)
            curr_angle_a, curr_angle_b = self.move_arm_old(curr_angle_a, curr_angle_b, segment_x, segment_y, l_a, l_b)

        return curr_angle_a, curr_angle_b

    def move_arm_old(self, start_angle_a, start_angle_b, target_x, target_y, l_a, l_b):
        """
        Calculate the angles for the robotic arm to move to the target point and send commands to the arms.
        """
        # Calculate the starting position
        x_start = l_a * math.cos(math.radians(start_angle_a)) + \
                  l_b * math.cos(math.radians(start_angle_a + start_angle_b))
        y_start = l_a * math.sin(math.radians(start_angle_a)) + \
                  l_b * math.sin(math.radians(start_angle_a + start_angle_b))

        # print(f"Starting Position: x = {x_start:.2f}, y = {y_start:.2f}")

        # Distance to the target
        d = math.sqrt(target_x ** 2 + target_y ** 2)

        if d > l_a + l_b:
            raise ValueError("Target is unreachable! Point is outside the arm's reach.")

        # Calculate angle_b (elbow angle)
        cos_angle_b = (target_x ** 2 + target_y ** 2 - l_a ** 2 - l_b ** 2) / (2 * l_a * l_b)
        angle_b_new = math.degrees(math.acos(cos_angle_b))

        # Calculate angle_a (base angle)
        phi = math.atan2(target_y, target_x)
        theta = math.atan2(l_b * math.sin(math.radians(angle_b_new)),
                           l_a + l_b * math.cos(math.radians(angle_b_new)))
        angle_a_new = math.degrees(phi - theta)

        print(f"Target Position: x = {target_x}, y = {target_y}")
        print(f"New Angles: angle_a = {angle_a_new:.2f}, angle_b = {angle_b_new:.2f}")

        # Calculate the degree changes
        delta_a = angle_a_new - start_angle_a
        delta_b = angle_b_new - start_angle_b

        global arm_a_err, arm_b_err
        compensated_delta_a = np.fix((delta_a + arm_a_err) / self.deg_per_step) * self.deg_per_step
        arm_a_err = - (compensated_delta_a - (delta_a + arm_a_err))
        compensated_delta_b = np.fix((delta_b + arm_b_err) / self.deg_per_step) * self.deg_per_step
        arm_b_err = - (compensated_delta_b - (delta_b + arm_b_err))

        message = f"comb_rot {int(bool(delta_a>0))} {abs(int(np.fix(delta_a * self.steps_per_deg)))} {int(bool(delta_b>0))} {abs(int(np.fix(delta_b * self.steps_per_deg)))} {10000} "
        self.serial_connection.write((message + '\n').encode())
        print(message)
        print(
            f"Moved arm to target ({target_x}, {target_y}) with new angles: A={angle_a_new:.2f}, B={angle_b_new:.2f},\
                angle rotated is A={delta_a:.2f} and B={delta_b:.2f}")
        angle_a_final = start_angle_a + compensated_delta_a
        angle_b_final = start_angle_b + compensated_delta_b
        return angle_a_final, angle_b_final

    def move_arm_new(self, start_angle_a, start_angle_b, target_x, target_y, target_z, l_a, l_b, num_segments=None):
        self.serial_connection.reset_input_buffer()
        # Using the angle of two arm to calculate initial end-effector position
        x0 = l_a * math.cos(math.radians(start_angle_a)) + \
             l_b * math.cos(math.radians(start_angle_a + start_angle_b))
        y0 = l_a * math.sin(math.radians(start_angle_a)) + \
             l_b * math.sin(math.radians(start_angle_a + start_angle_b))

        # Calculate target position using IK, returns the two target angle that we want to reach
        # target_angle_a, target_angle_b = self.calculate_ik(target_x, target_y, l_a, l_b)

        if num_segments is None:
            # Calculate the number of segments based on the distance to the target
            # num_segments would range from 1-20, equal to the cartesian distance in mm divided by 10
            # !!! Note that the number 10 is a hyperparameter that can influence the accuracy of the arm movement
            # If the robotic arm movements are not accurate, consider lowering the number
            # More segments will be created, movements will be more accurate at the cost of runtime
            num_segments = max(min(20, (int(np.floor(math.hypot(target_x - x0, target_y - y0))))//10), 1)
            print('Number of segments: ', num_segments)

        # this is our final command that we sent to esp32
        command_sequence = []

        # Store previous angles for delta calculation
        prev_angle_a = start_angle_a
        prev_angle_b = start_angle_b

        # loop through each line segment and generate motion command
        for i in range(1, num_segments + 1):
            global arm_z

            # this is the target x and y for this segment
            x = x0 + i * (target_x - x0) / num_segments
            y = y0 + i * (target_y - y0) / num_segments
            z = arm_z + i * (target_z - arm_z) / num_segments
            delta_z = z - arm_z

            # Calculate IK for intermediate point, comp means error is compensated and stored in global var
            comp_delta_a, comp_delta_b = self.calculate_ik(x, y, l_a, l_b)

            # Convert to steps, rounding shouldn't be needed here, but calculate_ik returns an angle for now
            steps_a = int(round(comp_delta_a * self.steps_per_deg))
            steps_b = int(round(comp_delta_b * self.steps_per_deg))
            steps_z = delta_z / self.z_mm_per_step

            # Generate Bresenham steps for this segment
            dir_a = 1 if steps_a > 0 else 0
            dir_b = 1 if steps_b > 0 else 0
            dir_z = 1 if steps_z > 0 else 0
            abs_a = int(abs(steps_a))
            abs_b = int(abs(steps_b))
            abs_z = int(abs(steps_z))

            # Bresenham algorithm for this segment, note that this is only linear movement in polar space
            # we can fix this later, or just set line segment above to a larger number, so we don't have to worry
            # about having motion inside each segment linear
            dx = abs_a
            dy = abs_b
            err = 0
            x_step = 0
            y_step = 0

            # Here we start with the direction for both arm for this segment movement
            local_sequence = "CMD " + str(int(dir_a)) + " " + str(int(dir_b)) + " " + str(int(dir_z)) + " "

            # while x_step < dx or y_step < dy:
            #     step_cmd = []
            #     err2 = 2 * err
            #
            #     if err2 > -dy:
            #         err -= dy
            #         x_step += 1
            #         step_cmd.append('A')
            #
            #     if err2 < dx:
            #         err += dx
            #         y_step += 1
            #         step_cmd.append('B')
            #
            #     # Calculate dynamic delay based on segment progress
            #     current_step = max(x_step, y_step)
            #     total_steps = max(dx, dy)
            #     delay = self.calculate_delay(current_step, total_steps)
            #
            #     # the command for this one step
            #     local_sequence += f"{''.join(step_cmd)}:{delay} "

            stepped_a = 0
            stepped_b = 0
            stepped_z = 0
            max_val = int(max(abs_a, abs_b, abs_z))
            for j in range(max_val):
                step_cmd = []

                # Handle primary axis
                if abs_a == max_val:
                    step_cmd.append("A")
                    # Check B
                    if abs_b > 0 and (max_val / abs_b) * stepped_b < j:
                        step_cmd.append("B")
                        stepped_b += 1
                    # Check Z
                    if abs_z > 0 and (max_val / abs_z) * stepped_z < j:
                        step_cmd.append("Z")
                        stepped_z += 1
                elif abs_b == max_val:
                    step_cmd.append("B")
                    # Check A
                    if abs_a > 0 and (max_val / abs_a) * stepped_a < j:
                        step_cmd.append("A")
                        stepped_a += 1
                    # Check Z
                    if abs_z > 0 and (max_val / abs_z) * stepped_z < j:
                        step_cmd.append("Z")
                        stepped_z += 1
                elif abs_z == max_val:
                    step_cmd.append("Z")
                    # Check A
                    if abs_a > 0 and (max_val / abs_a) * stepped_a < j:
                        step_cmd.append("A")
                        stepped_a += 1
                    # Check B
                    if abs_b > 0 and (max_val / abs_b) * stepped_b < j:
                        step_cmd.append("B")
                        stepped_b += 1
                else:  # Handle ties
                    ties = "ABZ"
                    if abs_a == max_val and abs_b == max_val and abs_z == max_val and max_val != 0:
                        step_cmd = ties

                local_sequence += f"{''.join(step_cmd)}:{5000} "

            # command_sequence.extend(local_sequence)
            prev_angle_a += comp_delta_a
            prev_angle_b += comp_delta_b

            # and we don't need to update global arm angle since it's done in calculate_ik
            # sending the motion command for this local segment, but esp32 will only execute once it has all segment
            command_str = "CMD ".join(local_sequence)
            print("Sending one segment...")

            # update global arm_z
            arm_z = z

            print(local_sequence)
            
            self.serial_connection.reset_input_buffer()  # Clear incoming buffer
            self.serial_connection.reset_output_buffer() # Clear outgoing buffer

            # sleep to avoid buffer overflow
            # !!! Note that the sleep time is a hyperparameter that can influence the of the arm movement
            # If the robotic arm movements are not accurate, consider increasing the sleep time
            time.sleep(0.15)  
            self.serial_connection.write((local_sequence + "\n").encode())
            self.serial_connection.flush()
            
        print("DONE sent")

        # remember to only use ASCII encoded character < 127, we use @@ here
        self.serial_connection.write(("DONE" + "\n").encode())


        # Build final command string
        # old code that combine each local segment and then them down to ~2000 char command to avoid blown out serial
        # instead I will try to send each local segment individually and then combine them on the esp32 side
        # this might be slower?
        """
        dir_a_final = 1 if (target_angle_a > start_angle_a) else 0
        dir_b_final = 1 if (target_angle_b > start_angle_b) else 0
        command_str = f"CMD {dir_a_final} {dir_b_final} " + " ".join(command_sequence)
        print("Sending in segment...")
        trunc_command_str = command_str[8:]
        prefix_command_str = command_str[:8]
        print(prefix_command_str)
        while trunc_command_str != "":
            if len(trunc_command_str) > 2100:
                end_index = trunc_command_str[2000:].find(" ") + 2000
                this_segment_command_str = trunc_command_str[:end_index]
                print(this_segment_command_str)
                trunc_command_str = trunc_command_str[end_index:]
                self.serial_connection.write((prefix_command_str + this_segment_command_str + '\n').encode())
            else:
                self.serial_connection.write((prefix_command_str + trunc_command_str + '\n').encode())
                trunc_command_str = ""

            response = ""
            while response != "@@ACTION COMPLETED":
                try:
                    print(response)
                    response = self.serial_connection.readline().decode().strip()
                    time.sleep(0.2)
                except:
                    pass
        print(command_str)

        """

        return prev_angle_a, prev_angle_b

    def calculate_ik(self, target_x, target_y, l_a, l_b):
        global arm_a_deg, arm_b_deg
        start_angle_a = arm_a_deg
        start_angle_b = arm_b_deg
        x_start = l_a * math.cos(math.radians(start_angle_a)) + \
                  l_b * math.cos(math.radians(start_angle_a + start_angle_b))
        y_start = l_a * math.sin(math.radians(start_angle_a)) + \
                  l_b * math.sin(math.radians(start_angle_a + start_angle_b))

        print(f"Starting Position: x = {x_start:.2f}, y = {y_start:.2f}")

        # Distance to the target
        d = math.sqrt(target_x ** 2 + target_y ** 2)

        if d > l_a + l_b:
            raise ValueError("Target is unreachable! Point is outside the arm's reach.")

        # Calculate angle_b (elbow angle)
        cos_angle_b = (target_x ** 2 + target_y ** 2 - l_a ** 2 - l_b ** 2) / (2 * l_a * l_b)
        angle_b_new = math.degrees(math.acos(cos_angle_b))

        # Calculate angle_a (base angle)
        phi = math.atan2(target_y, target_x)
        theta = math.atan2(l_b * math.sin(math.radians(angle_b_new)),
                           l_a + l_b * math.cos(math.radians(angle_b_new)))
        angle_a_new = math.degrees(phi - theta)

        print(f"Target Position: x = {target_x}, y = {target_y}")
        print(f"New Angles: angle_a = {angle_a_new:.2f}, angle_b = {angle_b_new:.2f}")

        # Calculate the degree changes
        delta_a = angle_a_new - start_angle_a
        delta_b = angle_b_new - start_angle_b

        global arm_a_err, arm_b_err
        compensated_delta_a = np.fix((delta_a + arm_a_err) / self.deg_per_step) * self.deg_per_step
        arm_a_err = compensated_delta_a - (delta_a + arm_a_err)
        compensated_delta_b = np.fix((delta_b + arm_b_err) / self.deg_per_step) * self.deg_per_step
        arm_b_err = compensated_delta_b - (delta_b + arm_b_err)


        arm_a_deg += compensated_delta_a
        arm_b_deg += compensated_delta_b

        return compensated_delta_a, compensated_delta_b

    def calculate_delay(self, current_step, total_steps):
        # Smooth trapezoidal velocity profile
        return 10000
        accel_steps = min(50, total_steps // 3)
        if current_step < accel_steps:
            return 10000 + int(9000 * (1 - current_step / accel_steps))
        elif current_step > total_steps - accel_steps:
            return 10000 + int(9000 * (1 - (total_steps - current_step) / accel_steps))
        return 10000
    
    def cam_calibration(self):
        global arm_a_deg, arm_b_deg, arm_z
        global dist_per_pix

        img_filename = image_capture(app.selected_port_cam.get())
        
        x,y=select_point(img_filename)

        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, -78, 150, arm_z, 150, 100)
        dist_here = self.ask_dist()
        print("Distance here: ", dist_here)
        time.sleep(5)

        dist_per_pix = 0.00259 * dist_here + 0.0313

        wid_fov = 240*dist_per_pix  #240px
        len_fov = 320*dist_per_pix   #320px
        #y_comp = -35, x_comp = 5
        print("x_comp:", x_comp, "y_comp:", y_comp)
        start_x = -cam_pos - wid_fov/2 + x_comp*dist_per_pix  #camara is tilted towards +x
        start_y = 150 - len_fov/2 + y_comp*dist_per_pix #camara is tilted towards -y due to the arm structure, need to be compensated

        target_x = start_x + y*dist_per_pix
        target_y = start_y + x*dist_per_pix
        print("start_x:", start_x, "start_y:", start_y)
        print("target_x:", target_x, "target_y:", target_y)

        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, target_x, target_y, arm_z, 150, 150)

    def raster(self, tl_x, tl_y, area_raw, delay, resolution_factor = 1):
        """
        start_X and start_Y are the coordinate in mm of the top left of the area,
        area is a rectangular grid (2D array) where 1 is area that the arm should move to.
        dist_per_pix is the real life distance in mm

        This function will have the arm trace any irregular shaped area represented as a 2D array.
        """
        global arm_a_deg, arm_b_deg, arm_z
        global dist_per_pix

        def adjust_raster_resolution(arr, factor):    #factor=1 means the original array. Greater the factor, lower the resolution
            """
            Reduces the resolution of a binary (0/1) NumPy array by a given factor.
            
            Parameters:
            - arr: np.ndarray of shape (x, y) with binary values (0,1).
            - factor: int, the downscaling factor (new shape will be roughly (x//factor, y//factor)).
            Returns:
            - np.ndarray of shape (x//factor, y//factor) with reduced resolution.
            """
            x, y = arr.shape
            new_x, new_y = x // factor, y // factor

            # Reshape into blocks of size (factor, factor)
            reshaped = arr[:new_x * factor, :new_y * factor].reshape(new_x, factor, new_y, factor)

            reduced = np.round(reshaped.mean(axis=(1, 3))).astype(int)  # Majority voting
        
            return reduced
        
        area = adjust_raster_resolution(area_raw, resolution_factor)
        #plot out the numpy array area, save as image
        plt.imshow(area, cmap='gray')
        plt.savefig('reduced_resolution_area.png')

        #area = area_raw
        print('original_area:', area_raw.shape)
        print('adjusted_area:', area.shape)
        #check whether area only has 1 and 0
        if not np.all(np.isin(area, [0, 1])):
            raise ValueError("Area array must contain only 0 and 1 values.")
        rows = len(area)  #240px
        cols = len(area[0])   #320px

        dist_per_pix_new = dist_per_pix * resolution_factor

        # Move to the initial position (top-left corner)
        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, tl_x, tl_y, arm_z, 150, 100)
        time.sleep(7)

        height_map = np.zeros((rows, cols))

        for row in range(rows):
            if row % 2 == 0:
                # Left-to-right movement
                col_range = range(cols)
            else:
                # Right-to-left movement (zigzag)
                col_range = range(cols - 1, -1, -1)

            for col in col_range:
                if area[row][col] == 1:
                    # target_x = tl_x + col * dist_per_pix_new
                    # target_y = tl_y - row * dist_per_pix_new

                    #assume arm_a_deg = arm_b_deg = 90
                    target_y = tl_y + col * dist_per_pix_new
                    target_x = tl_x + row * dist_per_pix_new

                    # Move to the next valid position
                    arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg,target_x, target_y, arm_z, 150, 100)
                    time.sleep(delay)
                    dist_here = self.ask_dist()
                    height_map[row][col] = dist_here

        scanning_z = arm_z

        plot_3d_height_map(height_map)
        print("saved")

        offset = 15

        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, tl_x, tl_y, arm_z, 150, 150)
        time.sleep(4)
        avg_h = np.average(height_map)
        for row in range(rows):
            if row % 2 == 0:
                # Left-to-right movement
                col_range = range(cols)
            else:
                # Right-to-left movement (zigzag)
                col_range = range(cols - 1, -1, -1)

            for col in col_range:
                if area[row][col] == 1:
                    # target_x = tl_x + col * dist_per_pix
                    # target_y = tl_y - row * dist_per_pix

                    #assume arm_a_deg = arm_b_deg = 90
                    target_y = tl_y + col * dist_per_pix_new
                    target_x = tl_x + row * dist_per_pix_new

                    # Move to the next valid position
                    target_z = scanning_z - height_map[row][col]+ offset
                    arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, target_x, target_y, target_z, 150, 150)
                    time.sleep(delay + 0.4)  # delay between each movearm
        self.move_z(10)
        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, -150, 150, arm_z, 150, 150)  #moving back to the default position to check movement offset

    def draw_grid(self):
        global arm_a_deg, arm_b_deg, arm_z
        self.move_arm_new(arm_a_deg, arm_b_deg, -150, 240, arm_z, 150, 150)
        time.sleep(4)
        for i in range(10):
            self.move_arm_new(arm_a_deg, arm_b_deg, 0, 240-(i*10), arm_z, 150, 150)
            time.sleep(3)
            self.move_arm_new(arm_a_deg, arm_b_deg, -150, 240 - (i * 10), arm_z, 150, 150)
            time.sleep(3)
            self.move_arm_new(arm_a_deg, arm_b_deg, -150, 240 - ((i+1) * 10), arm_z, 150, 150)
            time.sleep(1)

    def ask_dist(self):
        global arm_a_deg, arm_b_deg
        global last_dist
        verify_location = str(round(arm_a_deg, 4)) + str(round(arm_b_deg, 4))
        self.serial_connection.write(("get_dist" + verify_location + '\n').encode())
        return_message = ""
        while not(return_message.startswith("DIST " + verify_location)):
            return_message = self.serial_connection.readline().decode().strip()
            if return_message.startswith("DIST " + verify_location):
                try:
                    dist = float(return_message[return_message.rfind(" "):])
                except:
                    print("Error: Distance returned cannot be made into a float!")
        if dist >= 200 or dist <= 2:
            dist = last_dist
        last_dist = dist
        return dist
    
    def move_z(self, z_target):
        global arm_z, arm_a_deg, arm_b_deg
        l_a = 150
        l_b = 150

        x_start = l_a * math.cos(math.radians(arm_a_deg)) + \
                  l_b * math.cos(math.radians(arm_a_deg + arm_b_deg))
        y_start = l_a * math.sin(math.radians(arm_a_deg)) + \
                  l_b * math.sin(math.radians(arm_a_deg + arm_b_deg))

        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, x_start, y_start, arm_z + z_target, 150, 150)
    
    def adjust_z_tof(self, z_offset):
        global arm_z, arm_a_deg, arm_b_deg
        
        dist = self.ask_dist()

        print("Current distance: ", dist)
        print("Moving down by", dist - z_offset)
        self.move_z(z_offset-dist)



class SerialCommApp():
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Communication")

        # Variables
        self.selected_port = tk.StringVar()
        self.selected_port_cam = tk.StringVar()
        self.baudrate = 512000
        self.serial_connection = None

        self.var_list = ["arm_a_deg", "arm_b_deg", "arm_z","dist_per_pix", "arm_a_err", "arm_b_err", "test_times", "x_comp", "y_comp"]
        self.var_display_name_list = ["1st Arm Angle", "2nd Arm Angle", "z" ,"mm/pix", "1st joint degree error", "2nd joint degree error", "test time", "x_compensation", "y_compensation"]	
        
        self.mask_array = None
        
        global arm_a_deg, arm_b_deg, arm_z, dist_per_pix, arm_a_err, arm_b_err, x_comp, y_comp
        global test_times

        default_font = tkFont.nametofont("TkDefaultFont")
        default_font.config(family="Lekton", size=12)

        # root.option_add("*Background", "white")
        # root.option_add("*Frame.Background", "white")
        # root.option_add("*Label.Background", "white")
        # root.option_add("*Button.Background", "white")
        # root.option_add("*Entry.Background", "white")

        def apply_bg(widget, color):
            widget.configure(bg=color)
            if isinstance(widget, tk.Tk) or isinstance(widget, tk.Frame):
                for child in widget.winfo_children():
                    try:
                        apply_bg(child, color)
                    except:
                        pass


        # GUI Layout
        self.create_widgets()

        print(tkFont.families())

        apply_bg(self.root, "white")

    def create_widgets(self):
        # Create main frames

        left_frame = tk.Frame(self.root, borderwidth=4, relief=tk.RIDGE)
        left_frame.pack(side="left", fill="y", expand=0)
        left_frame.columnconfigure(0, minsize=300)

        right_frame = tk.Frame(self.root, borderwidth=4, relief=tk.RIDGE)
        right_frame.pack(side="right", fill="none", expand=0)

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # Left Panel
        self.create_left_panel(left_frame)

        # Right Panel (existing widgets)
        self.create_right_panel(right_frame)

    def create_left_panel(self, parent):
        # Create variable rows
        self.var_widgets = []
        for i in range(len(self.var_list)):
            row_frame = tk.Frame(parent)
            row_frame.grid(row=i, column=0, pady=2, sticky="EW")
            frame = tk.Frame(row_frame)
            frame.pack(fill="x")

            # Variable name label
            name_label = tk.Label(frame, text=self.var_display_name_list[i])
            name_label.pack(side="left")

            # Variable name label
            dash_label = tk.Label(frame, text=str((30-len(self.var_display_name_list[i])) * "-"))
            dash_label.pack(side="left")

            # Entry widget
            entry = tk.Entry(frame, width=12)
            entry.pack(side="right", padx=10)

            # Current value label
            current_value = str(globals()[self.var_list[i]])
            value_label = tk.Label(frame, text=current_value)
            value_label.pack(side="right")



            self.var_widgets.append({
                "label": value_label,
                "entry": entry,
                "var_name": self.var_list[i]
            })
        # Update button
        update_button = ttk.Button(parent, text="Update Variables", command=self.update_variables)
        update_button.grid(sticky="se")

        #capture button
        #store_mask(): Call capture_and_segment(), and store the returned mask array as self.mask_array
        #capture_and_segment(display=True): the segmented mask will be displayed in the guim otherwise will just be saved as shape.png
        capture_button = ttk.Button(parent, text="Image Capture", command=lambda: self.store_mask())   
        capture_button.grid(sticky="se")

        start_button = ttk.Button(parent, text="Start", command=lambda: self.start_workflow())
        start_button.grid(sticky="se")

        # Add some padding around all elements
        for child in parent.winfo_children():
            child.grid_configure(padx=5, pady=2)

    def create_right_panel(self, parent):
        # Port Selection
        port_label = tk.Label(parent, text="Select Port:")
        port_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")

        self.port_menu = ttk.Combobox(
            parent, textvariable=self.selected_port, state="readonly", width=15
        )
        self.port_menu.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        refresh_button = ttk.Button(parent, text="Refresh", command=self.refresh_ports)
        refresh_button.grid(row=0, column=2, padx=5, pady=5, sticky="w")

        # Connect Button
        self.connect_button = ttk.Button(parent, text="Connect", command=self.connect_serial)
        self.connect_button.grid(row=0, column=3, padx=5, pady=5, sticky="w")

        #---------------
        port_label_cam = tk.Label(parent, text="Select Camera Port:")
        port_label_cam.grid(row=1, column=0, padx=5, pady=5, sticky="w")

        self.port_menu_cam = ttk.Combobox(
            parent, textvariable=self.selected_port_cam, state="readonly", width=15
        )
        self.port_menu_cam.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        refresh_button = ttk.Button(parent, text="Refresh", command=self.refresh_ports)
        refresh_button.grid(row=1, column=2, padx=5, pady=5, sticky="w")

        # # Connect Button
        # self.connect_button = ttk.Button(parent, text="Connect", command=self.connect_cam)
        # self.connect_button.grid(row=1, column=3, padx=5, pady=5, sticky="w")

        #--------------

        # Message Input
        message_label = tk.Label(parent, text="Message:")
        message_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")

        self.message_entry = tk.Entry(parent, width=30)
        self.message_entry.grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky="w")

        send_button = ttk.Button(parent, text="Send", command=self.send_message)
        send_button.grid(row=2, column=3, padx=5, pady=5, sticky="w")

        # Output Text Box
        output_label = tk.Label(parent, text="Response:")
        output_label.grid(row=3, column=0, padx=5, pady=5, sticky="w")

        self.output_text = tk.Text(parent, width=50, height=10, state="disabled")
        self.output_text.grid(row=4, column=0, columnspan=4, padx=5, pady=5, sticky="w")


        # Refresh ports at startup
        self.refresh_ports()

    def capture_and_segment(self, display = False):
        print(self.selected_port_cam.get())
        img_filename = image_capture(self.selected_port_cam.get())

        #img_filename = 'test_multimask.jpg'

        masks = segmentation_main(img_filename)
        

        print('Segmentation finished, mask is generated')
        global updated_mask

        index = 0
        while True:
            mask = masks[index]
            index += 1
            updated_mask = edit_mask(img_filename, mask)

            if not np.array_equal(updated_mask, 'Next'):
                break
            if index >= len(masks):
                index = 0

        plt.imshow(updated_mask,cmap='gray')
        plt.savefig("shape.png", bbox_inches="tight", dpi=300)

        if display:
            display_mask("segmented_mask.png")

        global dist_per_pix
        global cam_pos
        global arm_a_deg, arm_b_deg, arm_z

        parse_and_execute_command(f"move_arm {-cam_pos} {150} {arm_z} {150} {100}", arm, updated_mask)
        cam_dist = arm.ask_dist()
        print(f"Camera distance: {cam_dist} mm")

        # cam_dist = 0
        # while cam_dist ==0:
        #     cam_dist = arm.ask_dist()
        #     print(f"Camera distance: {cam_dist} mm")
        #     #arm.serial_connection.reset_input_buffer()
        
        # mm/px
        dist_per_pix = 0.00259 * cam_dist + 0.0313

        print(f"Distance per pixel: {dist_per_pix} mm")

        np.save("mask.npy", updated_mask)

        return updated_mask
    
    def store_mask(self):
        self.mask_array = self.capture_and_segment(display=False)
    
    def update_variables(self):
        for widget in self.var_widgets:
            entry = widget["entry"]
            new_value = entry.get().strip()

            if new_value:
                var_name = widget["var_name"]
                current_value = globals()[var_name]

                try:
                    # Convert input to appropriate type
                    if isinstance(current_value, bool):
                        converted_value = new_value.lower() == "true"
                    elif isinstance(current_value, float):
                        converted_value = float(new_value)
                    elif isinstance(current_value, int):
                        converted_value = int(new_value)
                    else:
                        converted_value = str(new_value)

                    globals()[var_name] = converted_value
                    if isinstance(converted_value, float):
                        widget["label"].config(text=str(round(converted_value, 4)))
                    else:
                        widget["label"].config(text=str(converted_value))
                    entry.delete(0, tk.END)
                except ValueError:
                    pass

    def refresh_variables_display(self):  
        for i, widget in enumerate(self.var_widgets):
            var_name = widget["var_name"]
            current_value = globals()[var_name]
            if isinstance(current_value, float):
                current_value = round(current_value, 4)
            widget["label"].config(text=str(current_value))


    def refresh_ports(self):
        """Refresh the available serial ports."""
        global test_times
        test_times += 1
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_menu['values'] = ports
        self.port_menu_cam['values'] = ports
        if ports:
            self.port_menu.current(0)
            self.port_menu_cam.current(0)


    def connect_serial(self):
        """Connect to the selected serial port."""
        if self.selected_port.get():
            try:
                self.serial_connection = serial.Serial(
                    self.selected_port.get(), self.baudrate, timeout=1
                )
                arm.serial_connection = self.serial_connection
                messagebox.showinfo("Success", f"Connected to {self.selected_port.get()}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {e}")
        else:
            messagebox.showwarning("Warning", "No port selected!")

    def connect_cam(self):
        """Connect to the selected serial port."""
        if self.selected_port_cam.get():
            try:
                self.serial_connection = serial.Serial(
                    self.selected_port_cam.get(), self.baudrate, timeout=1
                )
                #arm.serial_connection = self.serial_connection
                messagebox.showinfo("Success", f"Connected to {self.selected_port.get()}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {e}")
        else:
            messagebox.showwarning("Warning", "No port selected!")

    def send_message(self):
        """Send a message to the ESP32 and display the response."""
        if self.serial_connection and self.serial_connection.is_open:
            message = self.message_entry.get()
            if message:
                if message.startswith("rotate_arm") or message.startswith("move_arm") or message.startswith("raster") or message.startswith("ask_dist") or message.startswith("calibrate") or message.startswith("z_tof") or message.startswith("move_z") or message.startswith("cam_calibration") or message.startswith("draw_grid"):
                    parse_and_execute_command(message, arm, self.mask_array)
                    #parse_and_execute_command(message, arm, np.load("mask.npy"))

                else:
                    try:
                        # Send the message
                        self.serial_connection.write((message + '\n').encode())

                        # Wait and read the response
                        response = self.serial_connection.readline().decode().strip()
                        if response:
                            self.display_output(response)
                        else:
                            self.display_output("No response received!")
                    except Exception as e:
                        messagebox.showerror("Error", f"Communication error: {e}")
            else:
                messagebox.showwarning("Warning", "Message cannot be empty!")
        else:
            messagebox.showerror("Error", "No active connection!")

    def start_workflow(self):
        global arm_a_deg, arm_b_deg

        arm_a_deg = 90
        arm_b_deg = 90
        self.store_mask()

        dist = arm.ask_dist()

        while dist <= 20 or dist >= 160:  # not possible value, therefore we ask again
            dist = arm.ask_dist()
        adjusted_dist = dist - 12
        print("adjusted_dist:", adjusted_dist)
        
        time.sleep(7)
        parse_and_execute_command(f"move_z -{adjusted_dist}", arm, updated_mask)

        time.sleep(30)
        parse_and_execute_command(f"raster", arm, updated_mask)

        time.sleep(3)
        parse_and_execute_command(f"move_z {adjusted_dist-10}", arm, updated_mask)  # keep -10 because after printing it should move up 10




        

    def read_and_wait(self):
        """
        Wait and read message from ESP32 until a message end token has been received.
        """
        current_msg = ""
        while not(current_msg.startswith("@@END")):
            current_msg = self.serial_connection.readline().decode().strip()
        if current_msg:
            self.display_output(current_msg)

    def display_output(self, message):
        """Display the response in the output text box."""
        self.output_text.config(state="normal")
        self.output_text.insert(tk.END, message + "\n")
        self.output_text.config(state="disabled")
        self.output_text.see(tk.END)

    def close_serial(self):
        """Close the serial connection if open."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        root.destroy()


def parse_and_execute_command(command, arm_controller, mask, wait=5000):
    """
    Parses a command string and executes the corresponding function.
    Args:
        command (str): The input command string.
        arm_controller (RoboticArmController): The controller object for the robotic arm.
        wait (int): Default wait time in microseconds.
    """
    try:
        if command.startswith("rotate_arm_a"):
            # Split the command into parts
            parts = command.split()
            if len(parts) != 3:
                print("Invalid command format. Expected: rotate_arm_a <degrees> <wait>")
                return

            # Extract arguments
            degrees = float(parts[1])  # Convert degrees to a float
            wait = int(parts[2])  # Convert wait time to an integer

            # Call the rotate_arm_a function with the extracted degrees
            arm_controller.rotate_arm_a(degrees, wait)
            print(f"rotate_arm_a called with {degrees} degrees and wait {wait} µs")

        elif command.startswith("rotate_arm_b"):
            # Split the command into parts
            parts = command.split()
            if len(parts) != 3:
                print("Invalid command format. Expected: rotate_arm_a <degrees> <wait>")
                return

            # Extract arguments
            degrees = float(parts[1])  # Convert degrees to a float
            wait = int(parts[2])  # Convert wait time to an integer

            # Call the rotate_arm_a function with the extracted degrees
            arm_controller.rotate_arm_b(degrees, wait)
            print(f"rotate_arm_b called with {degrees} degrees and wait {wait} µs")
        elif command.startswith("move_arm"):
            # Split the command into parts
            parts = command.split()
            if len(parts) != 6:
                print("Invalid command format. Expected: move_arm <target_x> <target_y> <length_a> <length_b>")
                return

            # Extract arguments
            #x_start = float(parts[1])
            #y_start = float(parts[2])
            x_target = float(parts[1])
            y_target = float(parts[2])
            z_target = float(parts[3])
            length_a = float(parts[4])
            length_b = float(parts[5])

            # Call the rotate_arm_a function with the extracted degrees
            global arm_a_deg, arm_b_deg
            arm_a_deg, arm_b_deg = arm_controller.move_arm_new(arm_a_deg, arm_b_deg, x_target, y_target, z_target, length_a, length_b)
        elif command.startswith("calibrate"):
            #testing the accuracy of the arm, let it move back and forth from 0,300 to -100,100 for <repeat> times
            parts = command.split()
            if len(parts) != 2:
                print("Invalid command format. Expected: calibrate <repeat>")
                return
            repeat = int(parts[1])
            for c in range(repeat):
                arm_a_deg, arm_b_deg = arm.move_arm_new(arm_a_deg, arm_b_deg, -100, 100, 0, 150, 150)
                time.sleep(10)
                arm_a_deg, arm_b_deg = arm.move_arm_new(arm_a_deg, arm_b_deg, 0, 300, 0, 150, 150)
                time.sleep(10)

        elif command.startswith("move_z"):
            parts = command.split()
            if len(parts) != 2:
                print("Invalid command format. Expected: move_z <z_target>")
                return
            z_target = float(parts[1])

            arm_controller.move_z(z_target)

        elif command.startswith("z_tof"):
            parts = command.split()
            if len(parts) != 2:
                print("Invalid command format. Expected: z_tof <z_offset>")
                return
            z_offset = float(parts[1])

            arm_controller.adjust_z_tof(z_offset)

        elif command.startswith("cam_calibration"):
            arm_controller.cam_calibration()

        elif command.startswith("raster"):
            if mask is None:
                print("Image is not captured and segmented - Test raster function will be executed")
                print("1111")
                grid = np.ones((9, 9))
                arm.raster(-170, 170, grid, 0.1, resolution_factor=1)
            else:
                print("Raster function will be executed with the segmented mask")
                # assume the pen is at position (-150, 150) when the image is captured (arm_a_deg=90, arm_b_deg=90)
                # move the tof to the position where the camera was

                print('raster-dist_per_pix:', dist_per_pix)

                global x_comp, y_comp

                wid_fov = len(mask)*dist_per_pix  #240px
                len_fov = len(mask[0])*dist_per_pix   #320px
                #y_comp = -35, x_comp = 5
                print("x_comp:", x_comp, "y_comp:", y_comp)
                #!! Increase x_comp will move the print towards +x direction (left-hand side)
                #   Increase y_comp will move the print towards +y direction (away from the printer)
                start_x = -cam_pos - wid_fov/2 + x_comp*dist_per_pix  #camara is tilted towards +x
                start_y = 150 - len_fov/2 + y_comp*dist_per_pix #camara is tilted towards -y due to the arm structure, need to be compensated
                #start_y = 150 - len_fov/2
                print('len_fov:', len_fov, 'wid_fov:', wid_fov, 'start_x:', start_x, 'start_y:', start_y)
                arm.raster(start_x, start_y, mask, 0.2, resolution_factor=8)
        elif command.startswith("ask_dist"):
            arm_controller.ask_dist()
        elif command.startswith("draw_grid"):
            arm_controller.draw_grid()

    except (ValueError, IndexError) as e:
        raise Exception(f"Error parsing command: {e}")

def update_gui():
    global app
    app.refresh_variables_display()
    app.root.after(1000, update_gui)

# Main application
if __name__ == "__main__":
    root = tk.Tk()
    app = SerialCommApp(root)
    arm = RoboticArmController(app.serial_connection)
    
    root.protocol("WM_DELETE_WINDOW", app.close_serial)  # Ensure serial is closed on exit

    update_gui()
    root.mainloop()
