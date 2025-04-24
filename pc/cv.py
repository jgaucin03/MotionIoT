import cv2
import numpy as np
import serial
import time
import glob
import os
import platform
import serial.tools.list_ports
import threading
import queue

def list_serial_ports():
    """List available serial ports based on OS"""
    system = platform.system()
    
    if system == "Darwin":  # macOS
        # Original Mac implementation
        ports = glob.glob('/dev/cu.*')
        return ports
    elif system == "Windows":
        # Windows implementation
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append(port.device)
        return ports
    else:  # Linux or other
        ports = glob.glob('/dev/tty[A-Za-z]*')
        return ports

def connect_bluetooth(port_name, is_dual_port=False, rx_port_name=None):
    """Connect to the selected Bluetooth device"""
    # Connect to the primary (TX) port
    tx_ser = serial.Serial(
        port=port_name,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    print(f"Connected to {port_name} (TX) at 9600 baud")
    
    # If dual port mode is enabled, connect to RX port as well
    rx_ser = None
    if is_dual_port and rx_port_name:
        try:
            rx_ser = serial.Serial(
                port=rx_port_name,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"Connected to {rx_port_name} (RX) at 9600 baud")
        except Exception as e:
            print(f"Warning: Failed to connect to RX port: {e}")
            rx_ser = None
    
    return tx_ser, rx_ser

def send_command(tx_ser, command):
    """Send command to TIVA C with extensive debugging"""
    print(f"DEBUG: Sending to {tx_ser.port}: '{command}'")
    
    # Clear input buffer first
    tx_ser.reset_input_buffer()
    
    # Send command with newline
    tx_ser.write((command + '\r\n').encode('utf-8'))
    print(f"DEBUG: Command bytes sent, waiting for response...")
    
    # Wait for response with a timeout (increased to 3 seconds)
    start_time = time.time()
    response = ""
    
    while time.time() - start_time < 3.0:  # 3-second timeout for debugging
        if tx_ser.in_waiting > 0:
            chunk = tx_ser.read(tx_ser.in_waiting).decode('utf-8', errors='replace')
            print(f"DEBUG: Read {len(chunk)} bytes: '{chunk}'")
            response += chunk
            if '\n' in chunk:  # Got a complete line
                break
        else:
            # Print status every half second
            if int((time.time() - start_time) * 2) % 2 == 0:  # Every 0.5 seconds
                print(f"DEBUG: Waiting... ({time.time() - start_time:.1f}s elapsed)")
            time.sleep(0.1)
    
    # Print timeout if it occurred
    if time.time() - start_time >= 3.0:
        print(f"DEBUG: Timeout after 3 seconds waiting for response from {tx_ser.port}")
    
    print(f"DEBUG: Final response: '{response.strip()}'")
    return response.strip()

# Message queue for received commands
received_messages = queue.Queue()

def receive_commands_thread(rx_ser):
    """Background thread to receive commands from the RX port"""
    if rx_ser is None:
        return
        
    print("Starting receiver thread...")
    while True:
        try:
            if rx_ser.in_waiting > 0:
                data = rx_ser.readline().decode('utf-8', errors='replace').strip()
                if data:
                    # Add timestamp to message
                    timestamp = time.strftime("%H:%M:%S", time.localtime())
                    received_messages.put(f"[{timestamp}] {data}")
                    print(f"Received: {data}")
            time.sleep(0.1)  # Small delay to reduce CPU usage
        except Exception as e:
            print(f"Error in receiver thread: {e}")
            break

def download_yolo_files():
    """Download YOLOv4-tiny model files if they don't exist"""
    # Define file paths
    weights_path = "yolov4-tiny.weights"
    config_path = "yolov4-tiny.cfg"
    names_path = "coco.names"
    
    # URLs for the files
    weights_url = "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights"
    config_url = "https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg"
    names_url = "https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names"
    
    # Download files if they don't exist
    if not os.path.exists(weights_path):
        print(f"Downloading {weights_path} (this may take a minute)...")
        import urllib.request
        urllib.request.urlretrieve(weights_url, weights_path)
    
    if not os.path.exists(config_path):
        print(f"Downloading {config_path}...")
        import urllib.request
        urllib.request.urlretrieve(config_url, config_path)
    
    if not os.path.exists(names_path):
        print(f"Downloading {names_path}...")
        import urllib.request
        urllib.request.urlretrieve(names_url, names_path)
    
    print("All model files ready!")
    return weights_path, config_path, names_path

def setup_yolo_net():
    """Set up YOLO neural network with appropriate backend based on OS"""
    # Download YOLO files if needed
    weights_path, config_path, names_path = download_yolo_files()
    
    # Load COCO class names
    with open(names_path, 'r') as f:
        classes = f.read().strip().split('\n')
    
    # Load YOLOv4-tiny
    print("Loading YOLOv4-tiny model...")
    net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
    
    # Set backend based on platform
    system = platform.system()
    if system == "Darwin":  # macOS
        # Set backend optimized for M1 Mac
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    elif system == "Windows":
        # Use CUDA if available on Windows, otherwise OpenCV CPU
        try:
            cv2.cuda.getCudaEnabledDeviceCount()
            has_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
        except:
            has_cuda = False
            
        if has_cuda:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            print("Using CUDA acceleration")
        else:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            print("CUDA not available, using CPU")
    else:  # Linux or other
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    
    # Get output layer names
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    
    return net, output_layers, classes

def list_cameras():
    """List available cameras and let user choose one"""
    system = platform.system()
    available_cameras = []
    
    # Windows-specific camera handling
    if system == "Windows":
        # Try forcing DirectShow backend (correct usage as second parameter)
        for i in range(5):  # Check first 5 indexes
            # Use DirectShow backend on Windows
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"Camera index {i} is available")
                    available_cameras.append(i)
            cap.release()
    else:
        # For Mac and other systems
        for i in range(10):  # Check first 10 indexes
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"Camera index {i} is available")
                    available_cameras.append(i)
                cap.release()
    
    if not available_cameras:
        print("No cameras detected. Using default camera index 0.")
        print("NOTE: Make sure your webcam is not in use by another application")
        print("      and check Windows camera privacy settings in Settings → Privacy → Camera")
        return 0
    
    if len(available_cameras) == 1:
        print(f"Only one camera detected (index {available_cameras[0]}). Using it.")
        return available_cameras[0]
    
    print("Available cameras:")
    for i, idx in enumerate(available_cameras):
        print(f"{i+1}. Camera index {idx}")
    
    try:
        selection = int(input("Select camera number: ")) - 1
        if 0 <= selection < len(available_cameras):
            return available_cameras[selection]
        else:
            print("Invalid selection, using default camera (index 0)")
            return 0
    except ValueError:
        print("Invalid input, using default camera (index 0)")
        return 0

def main():
    # Display OS information
    system = platform.system()
    print(f"Detected operating system: {system}")
    
    # List available ports
    all_ports = list_serial_ports()
    if not all_ports:
        print("No serial ports found.")
        return
    
    print("Available ports:")
    for i, port in enumerate(all_ports):
        print(f"{i+1}. {port}")
    
    # Windows specific dual-port handling
    is_dual_port = False
    rx_port = None
    tx_port = None
    
    if system == "Windows":
        print("\nOn Windows, HC-05 modules create two COM ports.")
        use_dual = input("Do you want to use both ports to send and receive? (y/n): ").lower().strip()
        
        if use_dual == 'y':
            is_dual_port = True
            
            try:
                print("\nSelect the TX port (for sending commands):")
                tx_selection = int(input("Enter port number: ")) - 1
                if tx_selection < 0 or tx_selection >= len(all_ports):
                    print("Invalid selection")
                    return
                tx_port = all_ports[tx_selection]
                
                print("\nSelect the RX port (for receiving commands):")
                rx_selection = int(input("Enter port number: ")) - 1
                if rx_selection < 0 or rx_selection >= len(all_ports):
                    print("Invalid selection")
                    return
                rx_port = all_ports[rx_selection]
            except ValueError:
                print("Please enter valid numbers")
                return
        else:
            # Single port mode
            try:
                selection = int(input("\nSelect the Bluetooth port number: ")) - 1
                if selection < 0 or selection >= len(all_ports):
                    print("Invalid selection")
                    return
                tx_port = all_ports[selection]
            except ValueError:
                print("Please enter a valid number")
                return
    else:
        # Mac OS or other - use single port mode
        try:
            selection = int(input("Select the Bluetooth port number: ")) - 1
            if selection < 0 or selection >= len(all_ports):
                print("Invalid selection")
                return
            tx_port = all_ports[selection]
        except ValueError:
            print("Please enter a valid number")
            return
    
    # Connect to Bluetooth module(s)
    try:
        tx_ser, rx_ser = connect_bluetooth(tx_port, is_dual_port, rx_port)
        
        # Start receiver thread if using dual ports
        if is_dual_port and rx_ser:
            receiver_thread = threading.Thread(target=receive_commands_thread, args=(rx_ser,), daemon=True)
            receiver_thread.start()
        
        # Test connection
        response = send_command(tx_ser, "PING")
        if response != "PONG":
            print("Warning: Unexpected response from device")
        
        # Setup YOLOv4-tiny neural network
        net, output_layers, classes = setup_yolo_net()
        
        # Let user select a camera
        print("Scanning for available cameras...")
        camera_index = list_cameras()
        print(f"Using camera with index {camera_index}")
        
        # Initialize selected webcam
        print("Attempting to access webcam...")
        
        # Use DirectShow backend on Windows (correct usage as second parameter)
        if platform.system() == "Windows":
            cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(camera_index)
        
        # Check if webcam opened successfully
        if not cap.isOpened():
            print("Failed to open webcam. Trying alternative methods...")
            
            # Try alternative methods with correct parameter order
            alternatives = [
                lambda: cv2.VideoCapture(camera_index),  # Standard method
                lambda: cv2.VideoCapture(camera_index, cv2.CAP_DSHOW),  # DirectShow
                lambda: cv2.VideoCapture(camera_index, cv2.CAP_MSMF),   # Media Foundation
                lambda: cv2.VideoCapture(0, cv2.CAP_DSHOW),  # Default camera with DirectShow
                lambda: cv2.VideoCapture(0)   # Default camera as last resort
            ]
            
            for attempt, alt_method in enumerate(alternatives):
                print(f"Attempt {attempt+1}...")
                cap = alt_method()
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        print("Camera accessed successfully!")
                        break
                    cap.release()
            
            if not cap.isOpened():
                print("Failed to open webcam after multiple attempts.")
                print("Please check:")
                print("1. Windows camera privacy settings (Settings → Privacy → Camera)")
                print("2. That your webcam is not being used by another application")
                print("3. That your webcam drivers are properly installed")
                return
        
        # Configure detection parameters
        min_confidence = 0.5  # Minimum confidence for person detection
        min_detections = 2    # Number of consecutive frames with detections to trigger alert
        consecutive_detections = 0
        consecutive_non_detections = 0
        
        # For tracking motion detection state
        people_detected = False
        last_detection_time = 0
        detection_cooldown = 3  # seconds
        
        print("Starting detection. Press 'q' to quit...")
        print("(Detection may take a moment to process each frame)")
        
        # For displaying received messages
        font = cv2.FONT_HERSHEY_SIMPLEX
        message_history = []
        max_messages = 5  # Maximum number of messages to display
        
        while True:
            # Read frame from webcam
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame. Check camera.")
                time.sleep(0.5)
                continue
            
            # Get frame dimensions
            height, width, channels = frame.shape
            
            # Create a blob and pass it through the network
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)
            
            # Initialize lists for detected objects
            class_ids = []
            confidences = []
            boxes = []
            
            # Process each output layer
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    
                    # Filter out weak predictions and non-person detections
                    if confidence > min_confidence and class_id == 0:  # Class 0 is person in COCO
                        # Object detected is a person
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        
                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            # Apply non-maximum suppression to remove redundant overlapping boxes
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, min_confidence, 0.4)
            
            # Count people after NMS
            people_count = len(indexes)
            
            # Draw bounding boxes
            frame_with_boxes = frame.copy()
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = f"Person: {confidences[i]:.2f}"
                    cv2.rectangle(frame_with_boxes, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame_with_boxes, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Get current time
            current_time = time.time()
            
            # Temporal filtering - require multiple consecutive detections
            if people_count > 0:
                consecutive_detections += 1
                consecutive_non_detections = 0
            else:
                consecutive_detections = 0
                consecutive_non_detections += 1
            
            # Display detection status
            cv2.putText(frame_with_boxes, f"People: {people_count} (Consecutive: {consecutive_detections})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Only trigger alert after multiple consecutive detections
            if consecutive_detections >= min_detections:
                if not people_detected or (current_time - last_detection_time > detection_cooldown):
                    print(f"Motion confirmed! {people_count} people in frame")
                    # Send PERSON:1 command to device
                    response = send_command(tx_ser, "1") # MODIFIED
                    
                    people_detected = True
                    last_detection_time = current_time
                    # Visual indicator
                    cv2.putText(frame_with_boxes, "ALERT: Motion Detected!", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif consecutive_non_detections >= min_detections:
                if people_detected and (current_time - last_detection_time > detection_cooldown):
                    print("No motion detected")
                    # Send command to device to clear detection # MODIFIED
                    response = send_command(tx_ser, "0")
                    
                    people_detected = False
            
            # Process received messages (if using dual ports)
            if is_dual_port:
                while not received_messages.empty():
                    msg = received_messages.get()
                    message_history.append(msg)
                    if len(message_history) > max_messages:
                        message_history.pop(0)  # Remove oldest message
                
                # Display received messages on frame
                y_pos = height - 150  # Start position from bottom
                cv2.rectangle(frame_with_boxes, (10, y_pos - 20), (width - 10, height - 10), (0, 0, 0), -1)
                
                for i, msg in enumerate(message_history):
                    cv2.putText(frame_with_boxes, msg, (20, y_pos + (i * 25)), 
                               font, 0.6, (255, 255, 255), 1)
            
            # Show the frame
            cv2.imshow("Security Camera", frame_with_boxes)
            
            # Break the loop if 'q' is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        tx_ser.close()
        if is_dual_port and rx_ser:
            rx_ser.close()
        
    except serial.SerialException as e:
        print(f"Error with serial port: {e}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


def main_step1():
    """
    Test Step 1: Basic UART Communication Only
    
    This test focuses solely on establishing and verifying UART communication
    with the Tiva C microcontroller. No camera or person detection is performed.
    """
    # Display OS information
    system = platform.system()
    print(f"Detected operating system: {system}")
    
    # List available ports
    all_ports = list_serial_ports()
    if not all_ports:
        print("No serial ports found.")
        return
    
    print("Available ports:")
    for i, port in enumerate(all_ports):
        print(f"{i+1}. {port}")
    
    # Select port
    try:
        selection = int(input("Select the Bluetooth port number: ")) - 1
        if selection < 0 or selection >= len(all_ports):
            print("Invalid selection")
            return
        selected_port = all_ports[selection]
    except ValueError:
        print("Please enter a valid number")
        return
    
    # Connect to Bluetooth module
    try:
        tx_ser, _ = connect_bluetooth(selected_port)
        
        print("\n==== UART TEST MODE ====")
        print("This test will only verify UART communication.")
        print("Commands:")
        print("  1 - Send PING command")
        print("  2 - Send STATUS command")
        print("  3 - Send LED:R command")
        print("  4 - Send LED:G command")
        print("  5 - Send LED:B command")
        print("  q - Quit test")
        
        # Main test loop
        while True:
            cmd = input("\nEnter command (1-5, q): ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '1':
                # Test with multiple PINGs to verify reliability
                for i in range(5):
                    print(f"\nPING test {i+1} of 5:")
                    response = send_command(tx_ser, "PING")
                    print(f"Received response: '{response}'")
                    success = response.startswith() == "PONG"
                    print(f"Test {'PASSED' if success else 'FAILED'}")
                    time.sleep(0.5)
            elif cmd == '2':
                response = send_command(tx_ser, "STATUS")
                print(f"Received status: '{response}'")
            elif cmd == '3':
                response = send_command(tx_ser, "LED:R")
                print(f"LED command response: '{response}'")
            elif cmd == '4':
                response = send_command(tx_ser, "LED:G")
                print(f"LED command response: '{response}'")
            elif cmd == '5':
                response = send_command(tx_ser, "LED:B")
                print(f"LED command response: '{response}'")
            else:
                print("Invalid command. Please try again.")
        
        # Clean up
        tx_ser.close()
        print("Test complete. UART port closed.")
        
    except serial.SerialException as e:
        print(f"Error with serial port: {e}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

def main_step2():
    """
    Test Step 2: UART + Person Detection Commands Only
    
    This test adds camera-based person detection but does not
    continuously send detection messages to the Tiva C.
    Instead, it lets you control when to send the detection commands.
    """
    # Display OS information
    system = platform.system()
    print(f"Detected operating system: {system}")
    
    # List available ports
    all_ports = list_serial_ports()
    if not all_ports:
        print("No serial ports found.")
        return
    
    print("Available ports:")
    for i, port in enumerate(all_ports):
        print(f"{i+1}. {port}")
    
    # Select port
    try:
        selection = int(input("Select the Bluetooth port number: ")) - 1
        if selection < 0 or selection >= len(all_ports):
            print("Invalid selection")
            return
        selected_port = all_ports[selection]
    except ValueError:
        print("Please enter a valid number")
        return
    
    # Connect to Bluetooth module
    try:
        tx_ser, _ = connect_bluetooth(selected_port)
        
        # Test connection
        response = send_command(tx_ser, "PING")
        if response != "PONG":
            print("Warning: Unexpected response from device. Continuing anyway...")
        
        # Setup YOLOv4-tiny neural network
        net, output_layers, classes = setup_yolo_net()
        
        # Let user select a camera
        print("Scanning for available cameras...")
        camera_index = list_cameras()
        print(f"Using camera with index {camera_index}")
        
        # Initialize selected webcam
        print("Attempting to access webcam...")
        if platform.system() == "Windows":
            cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(camera_index)
        
        # Check if webcam opened successfully
        if not cap.isOpened():
            print("Failed to open webcam. Please check permissions.")
            return
        
        # Configure detection parameters
        min_confidence = 0.5  # Minimum confidence for person detection
        
        print("\n==== DETECTION TEST MODE ====")
        print("This test will show person detection but requires manual commands.")
        print("Instructions:")
        print("  p - Send PERSON:1 (just '1') command (detected)")
        print("  n - Send PERSON:0 (just '0') command (not detected)")
        print("  s - Send STATUS command")
        print("  q - Quit test")
        print("\nPress any key to start detection view...")
        input()
        
        # For detection status display
        people_detected = False
        
        while True:
            # Read frame from webcam
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame. Check camera.")
                time.sleep(0.5)
                continue
            
            # Get frame dimensions
            height, width, channels = frame.shape
            
            # Create a blob and pass it through the network
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)
            
            # Initialize lists for detected objects
            class_ids = []
            confidences = []
            boxes = []
            
            # Process each output layer
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    
                    # Filter out weak predictions and non-person detections
                    if confidence > min_confidence and class_id == 0:  # Class 0 is person in COCO
                        # Object detected is a person
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        
                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            # Apply non-maximum suppression to remove redundant overlapping boxes
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, min_confidence, 0.4)
            
            # Count people after NMS
            people_count = len(indexes)
            
            # Draw bounding boxes
            frame_with_boxes = frame.copy()
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = f"Person: {confidences[i]:.2f}"
                    cv2.rectangle(frame_with_boxes, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame_with_boxes, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Display detection status
            cv2.putText(frame_with_boxes, f"People detected: {people_count}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Add instructional overlay
            cv2.putText(frame_with_boxes, "Manual control: p=detected, n=not detected, s=status, q=quit", 
                       (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            # Show current detection state
            status_color = (0, 255, 0) if people_detected else (0, 0, 255)
            status_text = "DETECTION: ACTIVE" if people_detected else "DETECTION: INACTIVE"
            cv2.putText(frame_with_boxes, status_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Show the frame
            cv2.imshow("Detection Test Mode", frame_with_boxes)
            
            # Check for key press
            key = cv2.waitKey(1) & 0xFF
            
            # Process keypress commands
            if key == ord('q'):
                break
            elif key == ord('p'):
                print("\nSending person detected command...")
                response = send_command(tx_ser, "PERSON:1")
                people_detected = True
                print(f"Response: {response}")
            elif key == ord('n'):
                print("\nSending no person detected command...")
                response = send_command(tx_ser, "PERSON:0")
                people_detected = False
                print(f"Response: {response}")
            elif key == ord('s'):
                print("\nSending status command...")
                response = send_command(tx_ser, "STATUS")
                print(f"Status response: {response}")
        
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        tx_ser.close()
        print("Test complete. UART port and camera closed.")
        
    except serial.SerialException as e:
        print(f"Error with serial port: {e}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()