import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import numpy as np
import math
import tkinter.font as tkFont
from PIL import Image, ImageTk



"""
We have a global arm_a and arm_b deg so we can store the current position of the robotic arm.
Note that this is always round to deg per step
We have a global arm_a and arm_b err so we can compensate for movement smaller than our step
This error is considered and updated everytime the arm is moved.
"""
arm_a_deg = 0.0
arm_b_deg = 0.0
dist_per_pix = 8
arm_a_err = 0.0
arm_b_err = 0.0
test_times = 0


class RoboticArmController:
    def __init__(self, serial_connection):
        self.serial_connection = serial_connection
        self.steps_per_revolution = 200  # Example: 200 steps per motor revolution (1.8° per step)
        self.gear_ratio = 4*4  # Reduction ratio (motor:arm = 1:4) and we are doing quarter microstepping so 4*4
        self.steps_per_deg = self.steps_per_revolution * self.gear_ratio / 360
        self.deg_per_step = 1 / self.steps_per_deg

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

    def move_arm_new(self, start_angle_a, start_angle_b, target_x, target_y, l_a, l_b):
        # Using the angle of two arm to calculate initial end-effector position
        x0 = l_a * math.cos(math.radians(start_angle_a)) + \
             l_b * math.cos(math.radians(start_angle_a + start_angle_b))
        y0 = l_a * math.sin(math.radians(start_angle_a)) + \
             l_b * math.sin(math.radians(start_angle_a + start_angle_b))

        # Calculate target position using IK, returns the two target angle that we want to reach
        # target_angle_a, target_angle_b = self.calculate_ik(target_x, target_y, l_a, l_b)

        # Generate linear path in Cartesian space, which will have x segment where x is the path distance in mm.
        # num_segments = max(min(90, int(np.floor(math.hypot(target_x - x0, target_y - y0)))), 5)
        num_segments = 20

        # this is our final command that we sent to esp32
        command_sequence = []
        # getting the global arm a and b angle
        global arm_a_err, arm_b_err

        # Store previous angles for delta calculation
        prev_angle_a = start_angle_a
        prev_angle_b = start_angle_b

        # loop through each line segment and generate motion command
        for i in range(1, num_segments + 1):

            # this is the target x and y for this segment
            x = x0 + i * (target_x - x0) / num_segments
            y = y0 + i * (target_y - y0) / num_segments

            # Calculate IK for intermediate point, comp means error is compensated and stored in global var
            comp_delta_a, comp_delta_b = self.calculate_ik(x, y, l_a, l_b)

            # Convert to steps, rounding shouldn't be needed here, but calculate_ik returns an angle for now
            steps_a = int(round(comp_delta_a * self.steps_per_deg))
            steps_b = int(round(comp_delta_b * self.steps_per_deg))

            # Generate Bresenham steps for this segment
            dir_a = 1 if steps_a > 0 else 0
            dir_b = 1 if steps_b > 0 else 0
            abs_a = abs(steps_a)
            abs_b = abs(steps_b)

            # Bresenham algorithm for this segment, note that this is only linear movement in polar space
            # we can fix this later, or just set line segment above to a larger number, so we don't have to worry
            # about having motion inside each segment linear
            dx = abs_a
            dy = abs_b
            err = 0
            x_step = 0
            y_step = 0

            # Here we start with the direction for both arm for this segment movement
            local_sequence = "CMD " + str(int(dir_a)) + " " + str(int(dir_b)) + " "

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

            stepped = 0
            for j in range(max(abs_a, abs_b)):
                step_cmd = []

                if abs_a == 0:
                    step_cmd.append("B")
                elif abs_b == 0:
                    step_cmd.append("A")
                elif abs_a > abs_b:
                    step_cmd.append("A")
                    if ((abs_a / abs_b) * stepped) < j:
                        step_cmd.append("B")
                        stepped += 1
                elif abs_b > abs_a:
                    step_cmd.append("B")
                    if ((abs_b / abs_a) * stepped) < j:
                        step_cmd.append("A")
                        stepped += 1
                else:
                    step_cmd.append("AB")

                local_sequence += f"{''.join(step_cmd)}:{5000} "

            # command_sequence.extend(local_sequence)
            prev_angle_a += comp_delta_a
            prev_angle_b += comp_delta_b

            # and we don't need to update global arm angle since it's done in calculate_ik
            # sending the motion command for this local segment, but esp32 will only execute once it has all segment
            command_str = "CMD ".join(local_sequence)
            print("Sending one segment...")
            print(local_sequence)
            self.serial_connection.write((local_sequence + "\n").encode())
            response = ""
            # wait for esp32 to finish reading and return completed
            # while response != "##ACTION COMPLETED":
            #     try:
            #         # print(response)
            #         response = self.serial_connection.readline().decode().strip()
            #         time.sleep(0.02)
            #     except:
            #         print("ERROR reading from esp32 during segment motion.")
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

    def raster(self, tl_x, tl_y, area, delay):
        """
        start_X and start_Y are the coordinate in mm of the top left of the area,
        area is a rectangular grid (2D array) where 1 is area that the arm should move to.
        dist_per_pix is the real life distance in mm

        This function will have the arm trace any irregular shaped area represented as a 2D array.
        """
        global arm_a_deg, arm_b_deg
        global dist_per_pix

        rows = len(area)
        cols = len(area[0])

        # Move to the initial position (top-left corner)
        arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, tl_x, tl_y, 150, 150)
        time.sleep(3)

        for row in range(rows):
            if row % 2 == 0:
                # Left-to-right movement
                col_range = range(cols)
            else:
                # Right-to-left movement (zigzag)
                col_range = range(cols - 1, -1, -1)

            for col in col_range:
                if area[row][col] == 1:
                    target_x = tl_x + col * dist_per_pix
                    target_y = tl_y - row * dist_per_pix

                    # Move to the next valid position
                    arm_a_deg, arm_b_deg = self.move_arm_new(arm_a_deg, arm_b_deg, target_x, target_y, 150, 150)
                    time.sleep(delay)


class SerialCommApp():
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Communication")

        # Variables
        self.selected_port = tk.StringVar()
        self.baudrate = 115200
        self.serial_connection = None

        self.var_list = ["arm_a_deg", "arm_b_deg", "dist_per_pix", "arm_a_err", "arm_b_err", "test_times"]
        self.var_display_name_list = ["1st Arm Angle", "2nd Arm Angle", "mm/pix", "1st joint degree error", "2nd joint degree error", "test time"]
        global arm_a_deg, arm_b_deg, dist_per_pix, arm_a_err, arm_b_err
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

        # Message Input
        message_label = tk.Label(parent, text="Message:")
        message_label.grid(row=1, column=0, padx=5, pady=5, sticky="w")

        self.message_entry = tk.Entry(parent, width=30)
        self.message_entry.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky="w")

        send_button = ttk.Button(parent, text="Send", command=self.send_message)
        send_button.grid(row=1, column=3, padx=5, pady=5, sticky="w")

        # Output Text Box
        output_label = tk.Label(parent, text="Response:")
        output_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")

        self.output_text = tk.Text(parent, width=50, height=10, state="disabled")
        self.output_text.grid(row=3, column=0, columnspan=4, padx=5, pady=5, sticky="w")


        # Refresh ports at startup
        self.refresh_ports()

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
        if ports:
            self.port_menu.current(0)


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

    def send_message(self):
        """Send a message to the ESP32 and display the response."""
        if self.serial_connection and self.serial_connection.is_open:
            message = self.message_entry.get()
            if message:
                if message.startswith("rotate_arm") or message.startswith("move_arm") or message.startswith("raster"):
                    parse_and_execute_command(message, arm)
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


def parse_and_execute_command(command, arm_controller, wait=5000):
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
            if len(parts) != 5:
                print("Invalid command format. Expected: move_arm <target_x> <target_y> <length_a> <length_b>")
                return

            # Extract arguments
            #x_start = float(parts[1])
            #y_start = float(parts[2])
            x_target = float(parts[1])
            y_target = float(parts[2])
            length_a = float(parts[3])
            length_b = float(parts[4])

            # Call the rotate_arm_a function with the extracted degrees
            global arm_a_deg, arm_b_deg
            arm_a_deg, arm_b_deg = arm_controller.move_arm_new(arm_a_deg, arm_b_deg, x_target, y_target, length_a, length_b)
        elif command.startswith("raster"):
            print("1111")
            grid = np.ones((10, 10))
            arm_controller.raster(-200, 200, grid, 0.5)
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
