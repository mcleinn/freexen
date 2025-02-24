import time
import argparse
import re
import rtmidi
from collections import defaultdict

MTS_SYSEX_SIZE = 408  # Expected .syx file size

def parse_file(file_path):
    """
    Parse a configuration file.
    
    Args:
        file_path (str): Path to the configuration file.
        is_detailed (bool): Whether the file contains detailed configuration.
        
    Returns:
        dict: Parsed configuration data.
    """
    with open(file_path, 'r') as file:
        # Read the file and remove empty lines
        lines = [line.strip() for line in file if line.strip()]

    # Use defaultdict for both cases to avoid KeyError
    config = defaultdict(dict)

    # Split into blocks by detecting headers
    current_header = None
    for line in lines:
        if line.startswith("[") and line.endswith("]"):  # Detect header
            current_header = line[1:-1]
            config[current_header] = {}
        elif current_header:  # Process key-value pairs under the current header
            match = re.match(r"(Key|Chan|Col)_(\d+)=(.+)", line)
            if match:
                field, index, value = match.groups()
                config[current_header].setdefault(int(index), {})[field] = int(value) if field != "Col" else value
            else:
                key, value = line.split("=")
                try:
                    config[current_header][key.strip()] = int(value.strip())
                except ValueError:
                    pass

    return dict(config)

    
def split_byte(byte_value):
    """
    Splits a byte into two 7-bit values for SysEx transmission.

    Args:
        byte_value (int): The byte value to split (0–255).

    Returns:
        list: Two 7-bit values [MSB, LSB].
    """
    return [byte_value & 0x7F, (byte_value >> 7) & 0x7F]
    
def send_ltn_message(integrated_data, midi_out):
    """
    Sends SysEx messages based on integrated data.

    Args:
        integrated_data (dict): The integrated data structure containing board and entry information.
    """

    for board_name, entries in integrated_data.items():
        board_number = int(board_name[5:])
        for control_number, entry in entries.items():
            # Extract color components in BGR order from Col
            if not isinstance(entry, dict):
                print(f"Skipping invalid entry: {entry} (type: {type(entry)})")
                continue  # Skip non-dictionaries

            col = str(entry["Col"]).lstrip("#")
            red = int(col[0:2], 16)
            green = int(col[2:4], 16)
            blue = int(col[4:6], 16)

            # Reorder to BGR and split each into two 7-bit values
            value1_split = split_byte(blue)
            value2_split = split_byte(red)
            value3_split = split_byte(green)

            # Construct SysEx message
            sysex_message = (
                [0xF0, 0x7D, 1, board_number & 0x7f, control_number & 0x7F, entry["Chan"] & 0x7F, entry["Key"] & 0x7f] +  # SysEx header and InputKey
                value1_split + value2_split + value3_split +  # Split color values
                [0xF7]  # End of SysEx
            )

            # Send the SysEx message
            midi_out.send_message(sysex_message)
            print(
                f"Sent SysEx to board {board_number} control {control_number}: "
                f"{', '.join(f'0x{byte:02X}' for byte in sysex_message)}"
            )
            time.sleep(0.001) 

        
def load_syx_file(filename):
    """
    Reads a .syx file and ensures it has the correct length of 408 bytes.
    :param filename: Path to the .syx file
    :return: List of hex values representing the SysEx message, or None if invalid.
    """
    try:
        # Check file size before loading
        if os.path.getsize(filename) != MTS_SYSEX_SIZE:
            print(f"Error: File '{filename}' is not {MTS_SYSEX_SIZE} bytes.")
            return None

        with open(filename, "rb") as file:
            syx_data = list(file.read())  # Read binary file into a list of integers
        
        print(f"Successfully loaded '{filename}', {len(syx_data)} bytes.")
        return syx_data

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None
    except Exception as e:
        print(f"Error reading file: {e}")
        return None

def send_syx_message(syx_data, midi_out):
    """
    Sends a SysEx message using rtmidi.
    :param midi_out: rtmidi.MidiOut instance
    :param syx_data: List of hex values representing the SysEx data
    """
    if not syx_data:
        print("Error: No SysEx data to send.")
        return

    # Create full SysEx message (header + data + end)
    sysex_message = [0xF0, 0x7D, 2] + syx_data + [0xF7] 

    try:
        midi_out.send_message(sysex_message)
        print("SysEx message sent successfully.")
    except Exception as e:
        print(f"Error sending SysEx: {e}")

def list_midi_devices(midi):
    available_ports = midi.get_ports()

    if available_ports:
        print("Available MIDI ports:")
        for i, port in enumerate(available_ports):
            print(f"{i}: {port}")
    else:
        print("No MIDI ports available.")

    return available_ports

def nullable_int(value):    
    """Attempts to convert the given value to an integer, or returns None if it cannot be converted."""    
    try:        
        return int(value)    
    except (TypeError, ValueError):        
        return None

def main():
    parser = argparse.ArgumentParser(description="Parse configuration files and integrate data.")
    parser.add_argument("--ltn", help="Keyboard layout")
    parser.add_argument("--syx", help="Tuning configuration")
    parser.add_argument("--device", help="Port number")
    parser.add_argument("--program", type=nullable_int, help="Program number")
    parser.add_argument("--refresh", help="Refresh device")
    parser.add_argument("--list", help="Show list of devices")
    
    args = parser.parse_args()
    
    midi_out = rtmidi.MidiOut()
    
    if args.list: 
        list_midi_devices(midi_out)
        return
        
    midi_out.open_port(args.device)

    if args.program:
        print("Program change: " + str(args.program))
        midi_out.send_message([0xC0, args.program])   
        print("Sent program change.")
        
    if args.syx:
        syx_data = load_syx_file(args.syx, midi_out)
        print("Tuning data:")
        print(syx_data)       
        send_syx_message(midi_out)
        print("Sent SYX message.")

    if args.ltn:
        ltn_data = parse_file(args.ltn)
        print("Keyboard data:")
        print(ltn_data)
        send_ltn_message(ltn_data, int(args.device), midi_out)
        print("Sent LTN message.")
        
    if args.refresh:
        sysex_message = (
            [0xF0, 0x7D, 0, 0xF7]
        )
        midi_out.send_message(sysex_message)
        print("Sent refresh message.")
        
    midi_out.close_port()

if __name__ == "__main__":
    main()


