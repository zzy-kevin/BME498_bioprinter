import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

import math

arm_a_deg = 0
arm_b_deg = 0.0


class RoboticArmController:
    def __init__(self, serial_connection):
        self.serial_connection = serial_connection
        self.steps_per_revolution = 200  # Example: 200 steps per motor revolution (1.8° per step)
        self.gear_ratio = 4  # Reduction ratio (motor:arm = 1:4)

    def rotate_arm_a(self, degrees, wait):
        """
        Rotates the first arm (arm_a) by a certain degree.
        Args:
            degrees (float): The angle to rotate in degrees (positive for CW, negative for CCW).
            wait (int): The delay between PWM signals in microseconds.
        """
        direction = 0 if degrees > 0 else 1  # CW = 0, CCW = 1, REMEMBER THAT motor is upside down so this is inverted
        steps = abs(degrees) * self.steps_per_revolution * self.gear_ratio / 360
        message = f"rot_a {direction} {int(steps)} {wait}"
        self.serial_connection.write((message + '\n').encode())

    def rotate_arm_b(self, degrees, wait):
        """
        Rotates the second arm (arm_b) by a certain degree.
        Args:
            degrees (float): The angle to rotate in degrees (positive for CW, negative for CCW).
            wait (int): The delay between PWM signals in microseconds.
        """
        direction = 0 if degrees > 0 else 1  # CW = 0, CCW = 1
        steps = abs(degrees) * self.steps_per_revolution * self.gear_ratio / 360
        message = f"rot_b {direction} {int(steps)} {wait}"
        self.serial_connection.write((message + '\n').encode())

    import math

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
        self.rotate_arm_a(-delta_a, 5000)  # Adjust wait as needed
        self.rotate_arm_b(-delta_b, 5000)

        print(
            f"Moved arm to target ({target_x}, {target_y}) with new angles: A={angle_a_new:.2f}, B={angle_b_new:.2f},\
                angle rotated is A={delta_a:.2f} and B={delta_b:.2f}")
        return angle_a_new, angle_b_new


class SerialCommApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Serial Communication")

        # Variables
        self.selected_port = tk.StringVar()
        self.baudrate = 115200
        self.serial_connection = None

        # GUI Layout
        self.create_widgets()

    def create_widgets(self):
        # Port Selection
        port_label = ttk.Label(self.root, text="Select Port:")
        port_label.grid(row=0, column=0, padx=5, pady=5)

        self.port_menu = ttk.Combobox(
            self.root, textvariable=self.selected_port, state="readonly", width=15
        )
        self.port_menu.grid(row=0, column=1, padx=5, pady=5)

        refresh_button = ttk.Button(self.root, text="Refresh", command=self.refresh_ports)
        refresh_button.grid(row=0, column=2, padx=5, pady=5)

        # Connect Button
        self.connect_button = ttk.Button(self.root, text="Connect", command=self.connect_serial)
        self.connect_button.grid(row=0, column=3, padx=5, pady=5)

        # Message Input
        message_label = ttk.Label(self.root, text="Message:")
        message_label.grid(row=1, column=0, padx=5, pady=5)

        self.message_entry = ttk.Entry(self.root, width=30)
        self.message_entry.grid(row=1, column=1, columnspan=2, padx=5, pady=5)

        send_button = ttk.Button(self.root, text="Send", command=self.send_message)
        send_button.grid(row=1, column=3, padx=5, pady=5)

        # Output Text Box
        output_label = ttk.Label(self.root, text="Response:")
        output_label.grid(row=2, column=0, padx=5, pady=5)

        self.output_text = tk.Text(self.root, width=50, height=10, state="disabled")
        self.output_text.grid(row=3, column=0, columnspan=4, padx=5, pady=5)

        # Refresh ports at startup
        self.refresh_ports()

    def refresh_ports(self):
        """Refresh the available serial ports."""
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
                if message.startswith("rotate_arm") or message.startswith("move_arm"):
                    print("aa")
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
                print("Invalid command format. Expected: rotate_arm_a <degrees> <wait>")
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
    except (ValueError, IndexError) as e:
        raise Exception(f"Error parsing command: {e}")


# Main application
if __name__ == "__main__":
    root = tk.Tk()
    app = SerialCommApp(root)
    arm = RoboticArmController(app.serial_connection)

    root.protocol("WM_DELETE_WINDOW", app.close_serial)  # Ensure serial is closed on exit
    root.mainloop()
