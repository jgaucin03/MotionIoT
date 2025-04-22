import serial
import time
import sys
import glob
import threading
import binascii
import os

def clear_screen():
    """Clear the terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')

def list_serial_ports():
    """List available serial ports on various platforms"""
    if sys.platform.startswith('darwin'):  # macOS
        ports = glob.glob('/dev/cu.*')
        return ports
    elif sys.platform.startswith('win'):  # Windows
        from serial.tools import list_ports
        return [port.device for port in list_ports.comports()]
    elif sys.platform.startswith('linux'):  # Linux
        return glob.glob('/dev/tty[A-Za-z]*')
    else:
        print("Unsupported platform")
        return []

def read_from_serial(ser, stop_event):
    """Thread function to continuously read from serial port"""
    while not stop_event.is_set():
        if ser.in_waiting:
            try:
                # Read raw bytes first for diagnostics
                raw_data = ser.read(ser.in_waiting)
                
                # Print hex representation for debugging
                hex_data = binascii.hexlify(raw_data).decode('utf-8')
                print(f"\nReceived (hex): {' '.join([hex_data[i:i+2] for i in range(0, len(hex_data), 2)])}")
                
                # Then decode as text
                text_data = raw_data.decode('utf-8', errors='replace')
                if text_data:
                    print(f"Received (text): {text_data}")
                
                # Re-display prompt
                print("\nEnter command: ", end='', flush=True)
            except Exception as e:
                print(f"\nError reading serial: {e}")
                print("\nEnter command: ", end='', flush=True)
        time.sleep(0.1)

def send_with_delay(ser, message):
    """Send message with minimal delay"""
    print(f"Sending: {message!r}")
    
    # First clear any pending input
    ser.reset_input_buffer()
    
    # Send full message at once (no character-by-character delay)
    ser.write((message + '\r\n').encode('utf-8'))
    
    # Print the hex representation of what we sent
    hex_sent = ' '.join([f"{ord(c):02x}" for c in message + '\r\n'])
    print(f"Sent (hex): {hex_sent}")

def main():
    clear_screen()
    print("TIVA C Bluetooth Communication Tool (9600 baud)")
    print("-----------------------------------------------")
    
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
    
    # Connect at 9600 baud
    try:
        ser = serial.Serial(
            port=selected_port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"Connected to {selected_port} at 9600 baud")
        
        # Create a thread to read from serial port
        stop_event = threading.Event()
        serial_thread = threading.Thread(target=read_from_serial, args=(ser, stop_event))
        serial_thread.daemon = True
        serial_thread.start()
        
        # Print menu
        print("\nCommands:")
        print("  PING       - Test connection (should respond with PONG)")
        print("  LED:R      - Set LED to Red")
        print("  LED:G      - Set LED to Green")
        print("  LED:B      - Set LED to Blue")
        print("  LED:Y      - Set LED to Yellow")
        print("  LED:P      - Set LED to Purple")
        print("  LED:C      - Set LED to Cyan")
        print("  LED:W      - Set LED to White") 
        print("  LED:OFF    - Turn LED off")
        print("  ECHO:text  - Echo back text")
        print("  STATUS     - Check TIVA C status")
        print("  HELP       - Show all available commands")
        print("  CLEAR      - Clear screen")
        print("  EXIT       - Exit program")
        
        # Command loop
        while True:
            try:
                cmd = input("\nEnter command: ").strip()
                
                if cmd.upper() == "EXIT":
                    break
                elif cmd.upper() == "CLEAR":
                    clear_screen()
                    continue
                elif cmd.upper() in ["PING", "STATUS", "HELP"] or \
                     cmd.upper().startswith("LED:") or \
                     cmd.upper().startswith("ECHO:"):
                    send_with_delay(ser, cmd)
                else:
                    print("Unknown command. Try PING, LED:R, LED:G, LED:B, STATUS, HELP, or EXIT")
                
                # Small delay to allow response to come in
                time.sleep(0.5)
            
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except serial.SerialException as e:
                print(f"Error with serial port: {e}")
                break
    
    except serial.SerialException as e:
        print(f"Error connecting to {selected_port}: {e}")
    finally:
        # Clean up
        if 'ser' in locals() and hasattr(ser, 'is_open') and ser.is_open:
            stop_event.set()
            time.sleep(0.5)
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()