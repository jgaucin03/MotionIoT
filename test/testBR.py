import serial
import time
import sys
import glob
import binascii

def list_serial_ports():
    """List available serial ports on macOS"""
    if sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/cu.*')
        return ports
    else:
        print("This script is designed for macOS")
        return []

def send_with_delay(ser, message):
    """Send message with delay between characters"""
    print(f"Sending with delay: {message}")
    
    # First clear any pending input
    ser.reset_input_buffer()
    
    # Send each character with delay
    for char in message:
        ser.write(char.encode('utf-8'))
        time.sleep(0.01)  # 10ms delay between characters
    
    # Send line ending with delay
    ser.write('\r'.encode('utf-8'))
    time.sleep(0.01)
    ser.write('\n'.encode('utf-8'))
    
    # Print the hex representation of what we sent
    hex_sent = ' '.join([f"{ord(c):02x}" for c in message + '\r\n'])
    print(f"Sent (hex): {hex_sent}")

def test_baud_rate(port, rate):
    """Test a specific baud rate"""
    print(f"\n=== TESTING BAUD RATE: {rate} ===")
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        print(f"Connected to {port} at {rate} baud")
        
        # Clear buffer
        ser.reset_input_buffer()
        time.sleep(0.5)
        
        # Send PING command
        send_with_delay(ser, "PING")
        
        # Wait for response
        time.sleep(1.0)
        
        # Read response
        if ser.in_waiting:
            # Read raw bytes first for diagnostics
            raw_data = ser.read(ser.in_waiting)
            
            # Print hex representation for debugging
            hex_data = binascii.hexlify(raw_data).decode('utf-8')
            print(f"Received (hex): {' '.join([hex_data[i:i+2] for i in range(0, len(hex_data), 2)])}")
            
            # Then decode as text
            text_data = raw_data.decode('utf-8', errors='replace')
            print(f"Received (text): {text_data}")
        else:
            print("No response received")
        
        # Ask user for rating
        quality = input("Rate the response quality (0-5, 0=garbage, 5=perfect): ")
        
        # Close connection
        ser.close()
        print(f"Connection closed for {rate} baud")
        
        return int(quality) if quality.isdigit() else 0
    
    except Exception as e:
        print(f"Error testing {rate} baud: {e}")
        return 0

def main():
    # List all available ports
    all_ports = list_serial_ports()
    if not all_ports:
        print("No serial ports found.")
        return
    
    print("Available ports:")
    for i, port in enumerate(all_ports):
        print(f"{i+1}. {port}")
    
    # Let user select a port
    try:
        selection = int(input("Select a port number: ")) - 1
        if selection < 0 or selection >= len(all_ports):
            print("Invalid selection")
            return
        selected_port = all_ports[selection]
    except ValueError:
        print("Please enter a valid number")
        return
    except KeyboardInterrupt:
        print("\nExiting...")
        return
    
    # Baud rates to test
    baud_rates = [9600, 38400, 57600, 115200, 4800, 19200]
    results = {}
    
    print("\nWe'll test each baud rate one by one.")
    print("After each test, you'll need to manually forget and re-pair the HC-05 device.")
    
    for rate in baud_rates:
        print("\n" + "="*50)
        print(f"PREPARING TO TEST {rate} BAUD")
        print("="*50)
        
        print("\nPlease follow these steps:")
        print("1. Open Bluetooth settings on your Mac")
        print("2. If HC-05 is connected, disconnect and forget the device")
        print("3. Search for the HC-05 device and pair it again (PIN: 1234)")
        print("4. Wait for the pairing to complete")
        
        input("\nPress Enter when you've completed these steps and are ready to test...")
        
        # Test this baud rate
        quality = test_baud_rate(selected_port, rate)
        results[rate] = quality
    
    # Show results summary
    print("\n" + "="*50)
    print("BAUD RATE TEST RESULTS")
    print("="*50)
    
    for rate, quality in sorted(results.items(), key=lambda x: x[1], reverse=True):
        print(f"Baud Rate: {rate} - Quality Score: {quality}/5")
    
    # Recommend the best baud rate
    best_rate = max(results.items(), key=lambda x: x[1])[0]
    print(f"\nRecommended baud rate: {best_rate}")
    
    print("\nTest complete. You can now update your TIVA C and Python code to use the recommended baud rate.")

if __name__ == "__main__":
    main()