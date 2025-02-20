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
        self.gear_ratio = 4  # Reduction ratio (motor:arm = 1:4)
        self.steps_per_deg = self.steps_per_revolution * self.gear_ratio / 360
        self.deg_per_step = 1/ self.steps_per_deg

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

    def move_arm(self, start_angle_a, start_angle_b, target_x, target_y, l_a, l_b):
        """
        Calculate the angles for the robotic arm to move to the target point and send commands to the arms.
        """
        # Calculate the starting position
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



        #return new_angle_a, new_angle_b

        # Send commands to the arms to rotate
        #self.rotate_arm_a(-delta_a, 5000)  # Adjust wait as needed
        #self.rotate_arm_b(-delta_b, 5000)
        global arm_a_err, arm_b_err
        compensated_delta_a = np.fix((delta_a + arm_a_err) / self.deg_per_step) * self.deg_per_step
        arm_a_err = compensated_delta_a - (delta_a + arm_a_err)
        compensated_delta_b = np.fix((delta_b + arm_b_err) / self.deg_per_step) * self.deg_per_step
        arm_b_err = compensated_delta_b - (delta_b + arm_b_err)

        message = f"comb_rot {int(bool(delta_a>0))} {abs(int(np.fix(delta_a * self.steps_per_deg)))} {int(bool(delta_b>0))} {abs(int(np.fix(delta_b * self.steps_per_deg)))} {4000} "
        self.serial_connection.write((message + '\n').encode())
        print(message)
        print(
            f"Moved arm to target ({target_x}, {target_y}) with new angles: A={angle_a_new:.2f}, B={angle_b_new:.2f},\
                angle rotated is A={delta_a:.2f} and B={delta_b:.2f}")
        angle_a_final = start_angle_a + compensated_delta_a
        angle_b_final = start_angle_b + compensated_delta_b
        return angle_a_final, angle_b_final

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
        arm_a_deg, arm_b_deg = self.move_arm(arm_a_deg, arm_b_deg, tl_x, tl_y, 150, 150)
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
                    arm_a_deg, arm_b_deg = self.move_arm(arm_a_deg, arm_b_deg, target_x, target_y, 150, 150)
                    time.sleep(delay)


class SerialCommApp():
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Communication")

        # Variables
        self.selected_port = tk.StringVar()
        self.baudrate = 115200
        self.serial_connection = None

        # Read variables
        self.dummy_var1 = "Dummy 1"
        self.dummy_var2 = 100
        self.dummy_var3 = 3.14
        self.dummy_var4 = True
        self.dummy_var5 = "Test"

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
                    widget["label"].config(text=str(converted_value))
                    entry.delete(0, tk.END)
                except ValueError:
                    pass

    def refresh_variables_display(self):
        for i, widget in enumerate(self.var_widgets):
            var_name = widget["var_name"]
            current_value = globals()[var_name]
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
        while not(current_msg.startswith("##END")):
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
            arm_a_deg, arm_b_deg = arm_controller.move_arm(arm_a_deg, arm_b_deg, x_target, y_target, length_a, length_b)
        elif command.startswith("raster"):
            print("1111")
            grid = np.ones((8, 8))
            arm_controller.raster(-200, 200, grid)
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
