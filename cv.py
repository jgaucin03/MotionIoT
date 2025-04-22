import cv2
import numpy as np
import serial
import time
import glob
import os
import platform
import serial.tools.list_ports

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

def list_cameras():
    """List available cameras and let user choose one"""
    available_cameras = []
    for i in range(10):  # Check first 10 indexes
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"Camera index {i} is available")
                available_cameras.append(i)
            cap.release()
    
    if not available_cameras:
        print("No cameras detected")
        return 0
    
    if len(available_cameras) == 1:
        print(f"Only one camera detected (index {available_cameras[0]}). Using it.")
        return available_cameras[0]
    
    print("Available cameras:")
    for i, idx in enumerate(available_cameras):
        print(f"{i+1}. Camera index {idx}")
    
    selection = int(input("Select camera number: ")) - 1
    if 0 <= selection < len(available_cameras):
        return available_cameras[selection]
    else:
        print("Invalid selection, using default camera (index 0)")
        return 0

def connect_bluetooth(port_name):
    """Connect to the selected Bluetooth device"""
    ser = serial.Serial(
        port=port_name,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    print(f"Connected to {port_name} at 9600 baud")
    return ser

def send_command(ser, command):
    """Send command to TIVA C with better reliability"""
    print(f"Sending: {command}")
    
    # Clear input buffer first
    ser.reset_input_buffer()
    
    # Send command with newline
    ser.write((command + '\r\n').encode('utf-8'))
    
    # Wait for response with a timeout
    start_time = time.time()
    response = ""
    
    while time.time() - start_time < 0.5:  # 500ms timeout
        if ser.in_waiting > 0:
            response += ser.readline().decode('utf-8', errors='replace').strip()
            if response:  # Got something, break out
                break
        time.sleep(0.05)  # Small delay
    
    print(f"Response: {response}")
    return response

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
    
    # Let user select a port
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
        ser = connect_bluetooth(selected_port)
        
        # Test connection
        response = send_command(ser, "PING")
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
        cap = cv2.VideoCapture(camera_index)
        
        # Check if webcam opened successfully
        if not cap.isOpened():
            print("Failed to open webcam. Please check permissions.")
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
                    response = send_command(ser, "PERSON:1")
                    
                    people_detected = True
                    last_detection_time = current_time
                    # Visual indicator
                    cv2.putText(frame_with_boxes, "ALERT: Motion Detected!", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif consecutive_non_detections >= min_detections:
                if people_detected and (current_time - last_detection_time > detection_cooldown):
                    print("No motion detected")
                    # Send command to device to clear detection
                    response = send_command(ser, "PERSON:0")
                    
                    people_detected = False
            
            # Show the frame
            cv2.imshow("Security Camera", frame_with_boxes)
            
            # Break the loop if 'q' is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error with serial port: {e}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()