import rtmidi
import argparse
import time

def list_midi_devices():
    midi_out = rtmidi.MidiOut()
    available_ports = midi_out.get_ports()
    print("Available MIDI Output Devices:")
    for i, port in enumerate(available_ports):
        print(f"{i}: {port}")  # Include backend index
    return available_ports
    
def hex_to_rgb(hex_color):
    """Convert a hex color (e.g., FFFFFF) to an RGB tuple."""
    hex_color = hex_color.lstrip('#')  # Remove '#' if present
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    
def color_correct(rgb, gamma=2.2):
    """Perform color correction with scaling factors and gamma correction."""
    # Scaling factors based on luminous intensities
    scale_factors = {'R': 1.0, 'G': 1.0, 'B': 1.0}
    #scale_factors = {'R': 1, 'G': 1, 'B': 1} 

    # Apply scaling factors
    r = rgb[0] * scale_factors['R']
    g = rgb[1] * scale_factors['G']
    b = rgb[2] * scale_factors['B']
    
    # Normalize to 0-255 range
    r = min(255, max(0, r))
    g = min(255, max(0, g))
    b = min(255, max(0, b))

    # Apply gamma correction
    r = 255 * ((r / 255) ** (1 / gamma))
    g = 255 * ((g / 255) ** (1 / gamma))
    b = 255 * ((b / 255) ** (1 / gamma))
    
    # Return corrected RGB values as integers
    return (int(r), int(g), int(b))

def split_byte(byte):
    """Split a byte (0–255) into two 7-bit MIDI-safe chunks."""
    low_7_bits = byte & 0x7F
    high_7_bits = (byte >> 7) & 0x7F
    return [low_7_bits, high_7_bits]
    
def send_sysex_to_device(device_number, corrected_rgb, program):
    midi_out = rtmidi.MidiOut()
    available_ports = midi_out.get_ports()
    
    if device_number < 0 or (device_number) >= len(available_ports):
        print("Invalid device number. Please select a valid device number.")
        return
    
    midi_out.open_port(device_number)
    print(f"Sending SysEx messages to: {available_ports[device_number]}")
    
    try:
        if not program is None:
            print("Program: " + str(program))
            midi_out.send_message([0xC0, program])
        for board in range(5): 
            for control in range(56):  # Index 0 to 111
                # Example values to send (0–255 range)
                value1 = corrected_rgb[2]  # BLUE - First byte
                value2 = corrected_rgb[0]  # RED - Second byte
                value3 = corrected_rgb[1]  # GREEN - Third byte

                channel = 0
                key = 0
             
                # Split each byte into two 7-bit chunks
                value1_split = split_byte(value1)
                value2_split = split_byte(value2)
                value3_split = split_byte(value3)
            
                # Construct SysEx message
                sysex_message = (
                    [0xF0, 0x7D, 1, board & 0x7F, control & 0x7F, channel & 0x7F, key & 0x7F] +  # Start with index
                    value1_split + value2_split + value3_split +  # Split values
                    [0xF7]  # End of SysEx
                )
            
                # Send SysEx message
                midi_out.send_message(sysex_message)
                hex_sequence = " ".join(f"{byte:02X}" for byte in sysex_message)
                print("SysEx sent:", hex_sequence)
            
                time.sleep(0.001)

        # Save and Update
        sysex_message = ( [0xF0, 0x7D, 0, 0xF7] )
        midi_out.send_message(sysex_message);
        hex_sequence = " ".join(f"{byte:02X}" for byte in sysex_message)
        print("SysEx sent:", hex_sequence);        
        print("SysEx messages sent successfully.")
    except Exception as e:
        print(f"Error sending SysEx messages: {e}")
    finally:
        midi_out.close_port()

def nullable_int(value):    
    """Attempts to convert the given value to an integer, or returns None if it cannot be converted."""    
    try:        
       return int(value)    
    except (TypeError, ValueError):        
       return None

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Send SysEx messages to multiple devices.")
    parser.add_argument(
        "--device", 
        type=str, 
        help="Device numbers to connect"
    )
    parser.add_argument(
        "--rgb", 
        type=str, 
        default="000000", 
        help="RGB values to send, in hex format (default: 000000)."
    )

    parser.add_argument(
        "--program", 
        type=nullable_int, 
        help="Program to save to"
    )

    # Parse command-line arguments
    args = parser.parse_args()
    
    # Get device numbers from command line or prompt for input
    if not args.device:
        list_midi_devices()
        selected_device = input("Enter device number to connect to: ")
    else:
        selected_device = args.device   

    # Convert hex RGB to tuple
    input_rgb = hex_to_rgb(args.rgb)
    print("Input RGB:", input_rgb)
    
    # Perform color correction
    corrected_rgb = color_correct(input_rgb)
    print("Corrected RGB:", corrected_rgb)
    
    try:
        send_sysex_to_device(int(selected_device), corrected_rgb, args.program)
    except ValueError:
        print("Invalid input. Please enter a number.")


def nullable_int(value):    
    """Attempts to convert the given value to an integer, or returns None if it cannot be converted."""    
    try:        
       return int(value)    
    except (TypeError, ValueError):        
       return None
