import serial
import time



#ser = serial.Serial("COM4", 115200, timeout=5)

def image_capture(port):
    ser = serial.Serial(port, 115200, timeout=5)
    ser.dtr = False
    ser.rts = False
    image_data = bytearray()

    print("Waiting for image data...")

    ser.reset_input_buffer()

    # Print every line received and START mark
    while True:
        if ser.in_waiting > 0: 
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"DEBUG: Received -> {line}")  # Print every line from ESP32
            if line == "<START>": # start when <START> detected
                print("Start marker received! Receiving image...")
                break

    # Read binary image data
    while True:
        chunk = ser.read(1024)  #  Read binary data 
        if b"<END>" in chunk:  # Stop when <END> is detected
            image_data.extend(chunk.split(b"<END>")[0])  # Store only valid image data
            break
        image_data.extend(chunk)
        print(f"Received {len(chunk)} bytes... (Total: {len(image_data)} bytes)")

    # Generate a unique filename using the current timestamp
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"captured_image_{timestamp}.jpg"

    # Save the received binary image
    if len(image_data) > 0:
        with open(filename, "wb") as f:
            f.write(image_data)
        print(f"Image saved as {filename} (Size: {len(image_data)} bytes)")
    else:
        print("ERROR: No image data received!")

    # send back acknoledge to ESP32 to free memory
    ser.write(b"ACK\n")
    print("ACK sent to ESP32")
    # close port
    ser.close()

    return filename
