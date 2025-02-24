import re, os
import mtsespy as mts # type: ignore
import rtmidi  # type: ignore
from rtmidi.midiconstants import PROGRAM_CHANGE  # type: ignore
from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO, emit
from xenharmlib import EDOTuning
from xenharmlib import play
from xenharmlib import UpDownNotation
from collections import defaultdict

LTN_DIRECTORY = "../xen2/ltn"

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'  # Required by Flask-SocketIO
socketio = SocketIO(app)

MENU_ITEMS = [
	(1,  "12ET-bosanquet",    "12ET-bosanquet.default.ltn",     12),
    (2,  "12ET-harmonicaxis", "12ET-harmonicaxis.default.ltn",  12),
    (3,  "12ET-multimanual",  "12ET-multimanual.default.ltn",   12),
    (4,  "19ET-bosanquet",    "19ET-bosanquet.default.ltn",     19),
    (5,  "22ET-bosanquet",    "22ET-bosanquet.default.ltn",     22),
    (6,  "24ET-isomorphic",   "24ET-isomorphic.default.ltn",    24),
    (7,  "29ET-bosanquet",    "29ET-bosanquet.default.ltn",     29),
    (8,  "31ET-bosanquet",    "31ET-bosanquet.default.ltn",     31),
    (9,  "53ET-bosanquet",    "53ET-bosanquet.default.ltn",     53),
    (10, "56ET-polychromatic","56ET-polychromatic.default.ltn", 56)
]

KEYS_PER_BOARD = 56

ltn = {}
# Keep track of the most recently forced index (if any)
current_tuning = 0

@app.route("/")
def index():
    """
    Render our 'index.html' template and pass in MENU_ITEMS
    to dynamically generate <li> elements.
    """
    return render_template("index.html", menu_items=MENU_ITEMS)

@app.route("/menuSelection")
def menu_selection():
    """
    Called by the client via Ajax (GET /menuSelection?index=X)
    whenever a user picks a menu item.
    """
    global current_tuning
    idx = request.args.get("index", None)
    if idx is not None:
        chosen_index = int(idx)
        current_tuning = chosen_index  # for demo, store it or do something else
        print(f"Client selected menu item index = {chosen_index}")
        return f"You chose item index {chosen_index}"
    return "No index provided"

@app.route("/changeScale")
def change_scale():
    """
    Let the server (or user) force a menu item by calling 
    /changeScale?index=X 
    which then sends a 'changeScale' event to all connected clients
    via Socket.IO.
    """
    global current_tuning
    idx = request.args.get("index", None)
    if idx is not None:
        xen_layout_change(int(idx))
        print(f"Forcing layout change = {current_tuning}")
        socketio.emit("changeScale", current_tuning)  # Broadcast to all
        return f"Forced layout change = {current_tuning}"
        
    return "No index provided"
    
@app.route("/noteOn")
def note_on():
    keys = request.args.get("keys", None)
    color = request.args.get("color", "#ffcc00")

    if keys is not None:
        key_list = keystring_to_list(keys)
        xen_note_on(key_list, color)
        print(f"Sending note ON to client for {key_list}")
        socketio.emit("noteOn", [ key_list, color ])  # Broadcast to all
        return f"Sending note ON to client for {key_list}"
        
    return "No index provided"
    
@app.route("/noteOff")
def note_off():
    keys = request.args.get("keys", None)   
    if keys is not None:
        key_list = keystring_to_list(keys)
        xen_note_off(key_list)
        print(f"Sending note OFF to client for {key_list}")
        socketio.emit("noteOff", key_list)  # Broadcast to all
        return f"Sending note OFF to client for {key_list}"
        
    return "No index provided"
    
@app.route("/getLayout")
def get_layout(): 
    global current_tuning
    
    """
    Returns a nested dictionary in JSON (like a 'keyboard layout').
    For demonstration, we return a trivial example:
    """
    index_str = request.args.get("index", "0")
    index = int(index_str)

    layout_data = ltn[index + 1]
    
    # Inform XEN about layout change
    xen_layout_change(index)
    
    return jsonify(layout_data)

@socketio.on("connect")
def on_connect():
    print("A client connected via Socket.IO")

def split_byte(byte_value):
    """
    Splits a byte into two 7-bit values for SysEx transmission.

    Args:
        byte_value (int): The byte value to split (0–255).

    Returns:
        list: Two 7-bit values [MSB, LSB].
    """
    return [byte_value & 0x7F, (byte_value >> 7) & 0x7F]
    
def bytes_from_color(color):
    col = color.lstrip("#")
    red = int(col[0:2], 16)
    green = int(col[2:4], 16)
    blue = int(col[4:6], 16)

    # Reorder to BGR and split each into two 7-bit values
    value1_split = split_byte(blue)
    value2_split = split_byte(red)
    value3_split = split_byte(green)
    return value1_split, value2_split, value3_split
    
def xen_layout_change(index):
    global current_tuning
    
    pc_channel = 0
    program_change_message = [0xC0 | pc_channel, index]
    xen_out.send_message(program_change_message)
    print("Sent PC1 msg with value {index} to XEN.")
    current_tuning = index
    
def xen_note_on(key_list, color_string):
    for key in key_list:
        board_number, control_number = key
        value1_split, value2_split, value3_split = bytes_from_color(color_string)
        sysex_message = (
            [0xF0, 0x7D, 3, board_number & 0x7f, control_number & 0x7F] +  # SysEx header and InputKey
                value1_split + value2_split + value3_split +  # Split color values
            [0xF7]  # End of SysEx
        )
        xen_out.send_message(sysex_message)
    print(f"Sent NoteON msgs {key_list} to XEN.")
    
def xen_note_off(key_list):
    for key in key_list:
        board_number, control_number = key
        sysex_message = (
            [0xF0, 0x7D, 4, board_number & 0x7f, control_number & 0x7F, 0xF7]  # End of SysEx
        )
        xen_out.send_message(sysex_message)
    print(f"Sent NoteOFF msgs {key_list} to XEN.")
    
def keystring_to_list(numbers_str):
    try:
        return [tuple(map(int, pair.split('@'))) for pair in numbers_str.split(',') if '@' in pair]
    except ValueError as e:
        print(f"Error parsing string: {e}")
        return None  # or return []


def parse_file(file_path):
    with open(file_path, 'r') as file:
        # Read the file and remove empty lines
        lines = [line.strip() for line in file if line.strip()]

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
                config[current_header].setdefault(index, {})[field] = int(value) if field != "Col" else value
            else:
                key, value = line.split("=")
                try:
                    config[current_header][key.strip()] = value.strip()
                except ValueError:
                    pass

    return dict(config)
    
def calculate_pitches(ltn, edo, scale_size, n_edo):
    for board_name, entries in ltn.items():
        board_number = int(board_name[5:])
        for control_number, entry in entries.items():
            if not isinstance(entry, dict):
                #print(f"Skipping invalid entry: {entry} (type: {type(entry)})")
                continue 
                
            chan = int(entry["Chan"])
            key = int(entry["Key"])
            pitch = chan * scale_size + key
            ep = edo.pitch(pitch)
            note = n_edo.guess_note(ep)
            
            entry["Pitch"] = pitch
            entry["Note"] = note.short_repr
            entry["Class"] = ep.pc_index
            entry["Base"] = ep.bi_index
            #print( f"{control_number}: Chan {chan} Key {key} Pitch {pitch} Class {ep.pc_index} BaseInt {ep.bi_index} Note {note.short_repr}")


        
            
def list_midi_devices(midiin, print_ports):
    available_ports = midiin.get_ports()
    
    if print_ports:
        if available_ports:
            print("Available MIDI output ports:")
            for i, port in enumerate(available_ports):
                print(f"{i}: {port}")
        else:
            print("No MIDI output ports available.")

    return available_ports
    
def get_port(ports, port_name):
    try:
        matches = [i for i, s in enumerate(ports) if (s.startswith(port_name))]
        if matches:
            index = matches[0]
            print(f"{port_name} at position {index}")
        else:
            index = -1
            print(f"'{port_name}' not found in the list.")
    except ValueError:
        index = -1
        print(f"'{port_name}' not found in the list.")
    return index 
    
def pedal_callback(event, data=None):
    """
    Callback function for handling incoming pedal MIDI messages.
    """
    global current_tuning
    message, delta_time = event
    print(f"Pedal message {message[0]} received.")
    # Check if the message is a Control Change message
    if message[0] == PROGRAM_CHANGE: # ch 1
        pc_value = message[1]
        current_tuning = pc_value
        print(f"Forcing menu item = {pc_value}")
        socketio.emit("changeScale", pc_value)  # Broadcast to all
        return f"Forced menu item = {pc_value}"
        
def xen_callback(event, data):
    message, _ = event  # Extract MIDI message
    status, note, velocity = message[:3]  # Get first 3 bytes

    channel = status & 0x0F  # Extract MIDI channel (0-15)
    message_type = status & 0xF0  # Extract message type

    if message_type == NOTE_ON and velocity > 0:
        print(f"NOTE ON:  Channel {channel+1}, Note {note}, Velocity {velocity}")
        if key_number is not None:
            socketio.emit("noteOn", [[note, channel], "ffcc00"])  # Broadcast to all
            print("Sent note ON for keys " + keys + " to client")
        
    elif message_type == NOTE_OFF or (message_type == NOTE_ON and velocity == 0):
        print(f"NOTE OFF: Channel {channel+1}, Note {note}, Velocity {velocity}")
        if key_number is not None:
            socketio.emit("noteOff", [note, channel])  # Broadcast to all
            print("Sent note OFF for keys " + keys + " to client")
        
if __name__ == "__main__":
    # Open Pedals
    pedal_in = rtmidi.MidiIn()
    ports = list_midi_devices(pedal_in, True)
    pedal_portnr = get_port(ports, "MidiSport 1x1")
    if pedal_portnr >= 0:
        pedal_in.open_port(pedal_portnr) 
    pedal_in.set_callback(pedal_callback)
       
    # Open Xen In
    xen_in = rtmidi.MidiIn()
    ports = list_midi_devices(xen_in, False)
    xen_portnr = get_port(ports, "Teensy MIDI")
    if xen_portnr >= 0:
        xen_portnr.open_port(xen_portnr) 
    xen_in.set_callback(xen_callback)
         
    # Open Xen Out
    xen_out = rtmidi.MidiOut()
    ports = list_midi_devices(xen_out, False)
    xen_portnr = get_port(ports, "Teensy MIDI")
    if xen_portnr >= 0:
        xen_portnr.open_port(xen_portnr) 
        
    # Load LTN
    for id, label, ltn_file, edo_divisions in MENU_ITEMS:
        ltn_file_abs = os.path.join(LTN_DIRECTORY, ltn_file)
        if not os.path.exists(ltn_file_abs):
            sys.exit(f"Error: LTN File '{ltn_file_abs}' does not exist. Exiting program.")
            
        ltn[id] = parse_file(ltn_file_abs)    
        edo = EDOTuning(edo_divisions)
        n_edo = UpDownNotation(edo) 
        calculate_pitches(ltn[id], edo, edo_divisions, n_edo)
    
    # Listen on port 8080
    socketio.run(app, host="0.0.0.0", port=8080, debug=True)
