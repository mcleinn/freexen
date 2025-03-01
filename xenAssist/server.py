import re, os
import atexit
import signal
import time
import threading
import mtsespy as mts # type: ignore
import rtmidi  # type: ignore
import re
import json
from rtmidi.midiconstants import PROGRAM_CHANGE  # type: ignore
from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO, emit
from xenharmlib import EDOTuning
from xenharmlib import play
from xenharmlib import UpDownNotation
from collections import defaultdict

LTN_DIRECTORY = "../xen2/ltn"
NOTE_ON = 0x90
NOTE_OFF = 0x80

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
OCTAVE_OFFSET = 1

ltn = {}
divisions = {}
edos = {}
tuningByMidi = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
tuningByPitch = defaultdict(lambda: defaultdict(list))
mts_clients = 0
saved_scale = None

# Keep track of the most recently forced index (if any)
current_tuning = 7

notation_replacements = {
        '^':    '',
        '^^':   '',
        '^^^':  '',
        'vvv#': '',
        'vv#':  '',
        'v#':   '',
        '#':    '',
        '^#':   '',
        '^^#':  '',
        '^^^#': '',
        'vvvx': '',
        'vvx':  '',
        'vx':   '',
        'x':    '',
        '^x':   '',
        '^^x':  '',
        '^^^x': '',
        'v':    '',
        'vv':   '',
        'vvv':  '',
        '^^^b': '',
        '^^b':  '',
        '^b':   '',
        'b':    '',
        'vb':   '',
        'vvb':  '',
        'vvvb': '',
        '^^^bb':'',
        '^^bb': '',
        '^bb':  '',
        'bb':   '',
        'vbb':  '',
        'vvbb': '',
        'vvvbb':''
}

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
        socketio.emit("noteOn", [ key_list, color, True ])  # Broadcast to all
        return f"Sending note ON to client for {key_list}"
        
    return "No index provided"
    
@app.route("/noteOff")
def note_off():
    keys = request.args.get("keys", None)   
    if keys is not None:
        key_list = keystring_to_list(keys)
        xen_note_off(key_list)
        print(f"Sending note OFF to client for {key_list}")
        socketio.emit("noteOff", [ key_list, True ])  # Broadcast to all
        show_saved_scale()
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
    

@app.route('/rotateInterval')
def rotate_interval():
    global current_tuning, saved_scale
    
    # Get the pitches parameter (should be a JSON string)
    pitches_str = request.args.get('pitches')
    direction = int(request.args.get('direction'))
    
    pitch_list = json.loads(pitches_str) if pitches_str else []
    # Ensure all items are integers.
    pitch_array = [int(x) for x in pitch_list]
       
    # Log the incoming parameter
    print("saveInterval called with pitches:", pitch_array)  
    
    t = current_tuning + 1 
    if saved_scale is None:
        previous_scale = edos[t].scale(
            [edos[t].pitch(p) for p in pitch_list]
        )
    else: 
        previous_scale = saved_scale
        hide_saved_scale()
        
    if direction == -1 and len(previous_scale) > 1:
        saved_scale = previous_scale.rotated_down()
    if direction == 1 and len(previous_scale) > 1:
        saved_scale = previous_scale.rotated_up()
        
    show_saved_scale()
    # Return a stub response
    return jsonify({
        "status": "ok",
        "message": f"rotateInterval called with direction {direction}"
    })

@app.route('/saveInterval')
def save_interval():
    global current_tuning, saved_scale
    
    # Get the pitches parameter (should be a JSON string)
    pitches_str = request.args.get('pitches')
    
    pitch_list = json.loads(pitches_str) if pitches_str else []
    # Ensure all items are integers.
    pitch_array = [int(x) for x in pitch_list]
   
    # Log the incoming parameter
    print("saveInterval called with pitches:", pitch_array)
    
    t = current_tuning + 1
    new_saved_scale = edos[t].scale(
        [edos[t].pitch(p) for p in pitch_list]
    ) 
      
    if saved_scale is None:   
        print("No previous saved scale. New scale: ", saved_scale)
    else:
        print("There was a previous saved scale. New scale: ", saved_scale)
        hide_saved_scale()
    
    if not new_saved_scale is None:
        saved_scale = new_saved_scale
        show_saved_scale()
    
    # Return a stub response
    return jsonify({
        "status": "ok",
        "message": "saveInterval OK",
        "pitches": pitch_array
    })

@app.route('/sendSavedNotes')
def sendSavedNotes():
    show_saved_scale()
    
    return jsonify({
    "status": "ok",
    "message": "sendSavedNotes OK"
    })
    
def hide_saved_scale():
    global saved_scale
    if saved_scale is None:
        return 
    print("Hide the following scale: ", saved_scale)
    key_list = key_tuple_from_scale(saved_scale)
    xen_note_off(key_list)
    socketio.emit("noteOff", [ key_list, False ])
    saved_scale = None
    
def show_saved_scale():
    global saved_scale
    if saved_scale is None:
        return 
    print("Show the following scale: ", saved_scale)
    key_list = key_tuple_from_scale(saved_scale)
    print("Note string: ", key_list)
    socketio.emit("noteOn", [ key_list, "ffcc00", False ])
    xen_note_on(key_list, "ffcc00")
    
def key_tuple_from_scale(scale):
    if scale is None:
        return []

    keys = []
    for pitch in scale:
        result = get_tuple_from_pitch(pitch)  # This returns a list of tuples or None
        if result is not None:
            # Extend our 'keys' list by adding all tuples from 'result'
            keys.extend(result)
    return keys
    
def get_tuple_from_pitch(pitch):
    global current_tuning 
    t = current_tuning 
    if pitch.pitch_index in tuningByPitch[t]:
        print("Pitch "+str(pitch.pitch_index)+" found in current tuning.")
        print(tuningByPitch[t][pitch.pitch_index])
        return [
            (entry["Key"], entry["Chan"] - 1)
            for entry in tuningByPitch[t][pitch.pitch_index]
        ]
    else:
        print(list(tuningByPitch[t].items())[:10])
        print("Pitch "+str(pitch.pitch_index)+" NOT found in current tuning.")
    return None 
    
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
    global current_tuning, saved_scale
    
    old_scale = saved_scale
    hide_saved_scale()
    
    pc_channel = 0
    program_change_message = [0xC0 | pc_channel, index]
    if not xen_out.is_port_open():
        xen_out_open(xen_out)
    if xen_out.is_port_open():
        xen_out.send_message(program_change_message)
    print(f"Sent PC1 msg with value {index} to XEN.")
    current_tuning = index
    apply_frequency_list()
    
    if not old_scale is None:          
        saved_scale = old_scale.retune(edos[index + 1])
        print(f"Old scale: {old_scale} Retuned scale: {saved_scale}")
    
def xen_note_on(key_list, color_string):
    global current_tuning
    for key in key_list:
        midi_note, midi_channel = key
        try:
            layout_list = tuningByMidi[current_tuning][midi_channel+1][midi_note]
            for layout in layout_list:
                value1_split, value2_split, value3_split = bytes_from_color(color_string)
                sysex_message = (
                    [0xF0, 0x7D, 3, layout["Board"] & 0x7f, layout["Control"] & 0x7F] +  # SysEx header and InputKey
                        value1_split + value2_split + value3_split +  # Split color values
                    [0xF7]  # End of SysEx
                )
                if not xen_out.is_port_open():
                    xen_out_open(xen_out)
                if xen_out.is_port_open():
                    xen_out.send_message(sysex_message)
                print(f"Sent NoteON msgs {key_list} to XEN board {layout["Board"]}, control {layout["Control"]}. Layout list: {layout_list}.")
        except Exception:
            print(f"Could not find {midi_note}@{midi_channel} in tuning {current_tuning}.")
    
def xen_note_off(key_list):
    for key in key_list:
        midi_note, midi_channel = key
        try:
            layout_list = tuningByMidi[current_tuning][midi_channel+1][midi_note]
            for layout in layout_list:
                sysex_message = (
                    [0xF0, 0x7D, 4, layout["Board"] & 0x7f, layout["Control"] & 0x7F, 0xF7]  # End of SysEx
                )
                if not xen_out.is_port_open():
                    xen_out_open(xen_out)
                if xen_out.is_port_open():
                    xen_out.send_message(sysex_message)
                print(f"Sent NoteOFF msgs {key_list} to XEN board {layout["Board"]}, control {layout["Control"]}.")
        except Exception:
            print(f"Could not find {midi_note}@{midi_channel} in tuning {current_tuning}.")
    
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
    
def encode_notation(s):
    m = re.match(r'^([v\^]*)([A-Z])([#xb]+)?(\d+)?$', s)
    if not m: return s
    prefix, note, suffix, octave = m.groups()
    suffix = suffix or ''
    octave = octave or ''
    key = prefix + suffix
    return note + notation_replacements.get(key, '') + octave
    
def calculate_pitches(ltn, edo, number_divisions, n_edo, tuning_number):
    for board_name, entries in ltn.items():
        board_number = int(board_name[5:])
        for control_number, entry in entries.items():
            if not isinstance(entry, dict):
                #print(f"Skipping invalid entry: {entry} (type: {type(entry)})")
                continue 
            
            entry["Board"] = int(board_number)
            entry["Control"] = int(control_number)               
            midi_chan = int(entry["Chan"])
            midi_note = int(entry["Key"])
            tuningByMidi[tuning_number][midi_chan][midi_note].append(entry)
            
            pitch = (midi_chan + OCTAVE_OFFSET) * number_divisions + midi_note 
            ep = edo.pitch(pitch)
            note = n_edo.guess_note(ep)
            freq = edo.get_frequency(ep)
            
            entry["Pitch"] = pitch
            tuningByPitch[tuning_number][pitch].append(entry)
            
            entry["Note"] = note.short_repr
            entry["NoteUnicode"] = encode_notation(note.short_repr)
            entry["Class"] = ep.pc_index
            entry["Base"] = ep.bi_index
            entry["Freq"] = freq.to_float()
            
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
    
def get_port(ports, port_name, direction):
    try:
        matches = [i for i, s in enumerate(ports) if (s.startswith(port_name))]
        if matches:
            index = matches[0]
            print(f"{port_name} {direction} at position {index}")
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
        xen_layout_change(pc_value)
        current_tuning = pc_value
        print(f"Forcing menu item = {pc_value}")
        socketio.emit("changeScale", pc_value)  # Broadcast to all
        return f"Forced menu item = {pc_value}"
        
def xen_callback(event, data):   
    global current_tuning
    try:
        message, _ = event  # Extract MIDI message
        status, midi_note, velocity = message[:3]  # Get first 3 bytes

        midi_channel = status & 0x0F  # Extract MIDI channel (0-15)
        message_type = status & 0xF0  # Extract message type

        if message_type == NOTE_ON and velocity > 0:
            try:
                name = tuningByMidi[current_tuning][midi_channel+1][midi_note][0]["Note"]
                freq = tuningByMidi[current_tuning][midi_channel+1][midi_note][0]["Freq"]
                print(f"NOTE ON:  Channel {midi_channel+1}, Note {midi_note}, Velocity {velocity}, Tuning {current_tuning}, Note {name}, Freq {freq}")
            except KeyError:
                print(f"NOTE ON:  Channel {midi_channel+1}, Note {midi_note}, Velocity {velocity} - NOT in layout")
            socketio.emit("noteOn", [[[midi_note, midi_channel]], "ffcc00", True])  # Broadcast to all
            print(f"Sent note ON for keys {midi_note}@{midi_channel} to client")
            show_saved_scale()
            
        elif message_type == NOTE_OFF or (message_type == NOTE_ON and velocity == 0):
            print(f"NOTE OFF: Channel {midi_channel+1}, Note {midi_note}, Velocity {velocity}")
            socketio.emit("noteOff", [[[midi_note, midi_channel]], True])  # Broadcast to all
            print(f"Sent note OFF for keys {midi_note}@{midi_channel} to client")
            show_saved_scale()
                
        else: 
            print(f"Unknown message type {message_type}")
             
    except Exception as e:
        print(f"Exception in xen_callback: {e}")  # Catch and print errors
        
def apply_frequency_list():
    """
    Apply the frequency list to the corresponding MIDI notes.
    This function is triggered by specific MIDI Control Change messages.
    """
    global current_tuning
    offset = divisions[current_tuning + 1]
    edo = edos[current_tuning + 1]
    
    for midi_ch in range(0,16):
        mts.clear_note_filter_multi_channel(midi_ch)
        for midi_note in range(0,128):
            try:
                entry = tuningByMidi[current_tuning][midi_ch][midi_note][0]
                print(f"MIDI {midi_note}@{midi_ch} (in layout) p={entry["Pitch"]} f={entry["Freq"]}")
            
                mts.set_multi_channel_note_tuning(entry["Freq"], midi_note, midi_ch)
            except Exception:
                pitch = (midi_ch + OCTAVE_OFFSET) * offset + midi_note
                ep = edo.pitch(pitch)
                freq = edo.get_frequency(ep).to_float()
            
                #print(f"MIDI {midi_note}@{midi_ch} (NOT in layout) p={pitch} f={freq}")
                mts.set_multi_channel_note_tuning(freq, midi_note, midi_ch)

def pedal_in_open(pedal_in):
    ports = list_midi_devices(pedal_in, True)
    pedal_portnr = get_port(ports, "MidiSport 1x1", "In")
    if pedal_portnr >= 0:
        pedal_in.open_port(pedal_portnr)     

def xen_in_open(xen_in):
    ports = list_midi_devices(xen_in, False)
    xen_portnr = get_port(ports, "Teensy MIDI", "In")
    if xen_portnr >= 0:
        xen_in.open_port(xen_portnr) 
        
def xen_out_open(xen_out):
    ports = list_midi_devices(xen_out, False)
    xen_portnr = get_port(ports, "Teensy MIDI", "Out")
    if xen_portnr >= 0:
        xen_out.open_port(xen_portnr) 
        
def check_ports_periodically():
    while True:
        if not pedal_in.is_port_open():
            pedal_in_open(pedal_in)
        if not xen_in.is_port_open():
            xen_in_open(xen_in)
        socketio.sleep(10)  # eventlet or gevent friendly "sleep"
        
def check_mts_clients():
    global mts_clients
    old_mts_clients = mts_clients
    mts_clients = mts.get_num_clients()   # type: ignore
    
    if old_mts_clients != mts_clients: {
        print(f"Number of clients: {mts_clients}")
    }
    socketio.sleep(1)
    
_master_instance = None

def start_master():
    global _master_instance
    if _master_instance is not None:
        return  # Already started

    try:
        _master_instance = mts.Master()  # effectively calls __init__()
    except Exception as e:
        print(f"When trying to register master: {e}")
        # Ignore—someone else has already registered a master
        pass

def stop_master():
    global _master_instance
    if _master_instance is not None:
        _master_instance.__exit__(None, None, None)
        _master_instance = None


# If you also rely on signals (SIGINT, SIGTERM), you can manually handle them:
def handle_sigterm(signum, frame):
    stop_master()
    # If you want to fully exit the Python process:
    import sys
    sys.exit(0)
    
        
if __name__ == "__main__":  
    # Use atexit to ensure clean deregistration on normal interpreter exit
    atexit.register(stop_master)

    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, handle_sigterm)
        signal.signal(signal.SIGINT, handle_sigterm)
        
    start_master()
        
    # Open Pedals
    pedal_in = rtmidi.MidiIn()
    pedal_in_open(pedal_in)
    pedal_in.set_callback(pedal_callback) 
       
    # Open Xen In
    xen_in = rtmidi.MidiIn()
    xen_in_open(xen_in)
    xen_in.set_callback(xen_callback)
         
    # Open Xen Out
    xen_out = rtmidi.MidiOut()
    xen_out_open(xen_out)
        
    # Load LTN
    for id, label, ltn_file, edo_divisions in MENU_ITEMS:
        ltn_file_abs = os.path.join(LTN_DIRECTORY, ltn_file)
        if not os.path.exists(ltn_file_abs):
            sys.exit(f"Error: LTN File '{ltn_file_abs}' does not exist. Exiting program.")
            
        ltn[id] = parse_file(ltn_file_abs) 
        divisions[id] = edo_divisions
        edos[id] = EDOTuning(edo_divisions)
        n_edo = UpDownNotation(edos[id]) 
        calculate_pitches(ltn[id], edos[id], divisions[id], n_edo, id-1)

    socketio.start_background_task(check_mts_clients)
    socketio.start_background_task(check_ports_periodically)   

    # Listen on port 8080
    socketio.run(app, host="0.0.0.0", port=8080, debug=True, allow_unsafe_werkzeug=True)
