import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import signal
import sys
import time
import threading
import musicalbeeps
import queue

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
RETRY_DELAY = 1#seconds elapsed before trying to connect again
PLAY_SOUNDS = True

ser = None#serial connection handler
analog_values = []
digital_values = []
running = True
input_requested = False
first_request = False
data_incoming = False  

#beep player
player = musicalbeeps.Player(volume=0.3, mute_output=True)
current_note = None#avoid playing the same note over and over again
sound_queue = queue.Queue()

'''
the thread plays a specific beep sound based on the analog value read from
the serial port
'''
def play_sound():
    current_note = None

    while running:
        try:
            note = sound_queue.get(timeout=1)
            if note != current_note:
                current_note = note
                if note is not None:
                    player.play_note(note, 0.1)#longer duration generates delays on serial reading
        except queue.Empty:
            continue

'''
map the analog values to a specific note
'''
def map_and_queue_sound(value):
    if value < 1250:
        sound_queue.put('C')
    elif 1250 <= value < 2500:
        sound_queue.put('D')
    elif 2500 <= value < 3750:
        sound_queue.put('E')
    elif 3750 <= value < 5000:
        sound_queue.put('F')
    else:
        sound_queue.put(None)

'''
handler for sigkill 
'''
def signal_handler(sig, frame):
    global running, ser
    running = False
    if(ser):
        ser.reset_output_buffer()
        ser.reset_input_buffer()
        ser.close()
    print("\nStopping data acquisition...")
    plt.close('all')
    sys.exit(0)

'''
reads the data received from the serial connection
'''
def read_serial(ser):
    global running, input_requested, first_request, data_incoming
    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                data_incoming = True
                if input_requested and (line.startswith("A:") or line.startswith("D:")):
                    input_requested = False
                    print("Input cancelled due to resumed transmission.")

                if line.startswith("A:"):
                    print(line)
                    try:
                        analog_value = float(line.split(':')[1])
                        analog_values.append(analog_value)
                        if(PLAY_SOUNDS): map_and_queue_sound(analog_value)#play sound
                    except ValueError:
                        continue

                elif line.startswith("D:"):
                    print(line)
                    try:
                        digital_value = int(line.split(':')[1])
                        digital_values.append(digital_value)
                    except ValueError:
                        continue
                
                elif line.startswith("C:"):
                    print("Insert filter mode [raw] [moving average] [random noise]")
                    if not input_requested:
                        input_requested = True
                        first_request = True
                        threading.Thread(target=request_user_input, args=(ser,)).start()
                else:
                    input_requested = True
                    first_request = True
                    print(line)
            elif not first_request:
                print("Insert filter mode [raw] [moving average] [random noise]")
                if not input_requested:
                    input_requested = True
                    first_request = True
                    threading.Thread(target=request_user_input, args=(ser,)).start()
        
        except Exception as e:
            if not running:
                break
            print("Error reading serial data:", e)

'''
update the plots with new data (analog and digital)
'''
def update_plot(frame):
    ax1.clear()
    ax2.clear()

    # Analog data
    ax1.plot(analog_values, label='Analog Signal', color='blue')
    ax1.set_title('Analog Values')
    ax1.set_xlabel('Time [ms]')
    ax1.set_ylabel('Analog Value')
    ax1.set_ylim([0, 5000])
    ax1.legend()
    ax1.grid()

    # Digital data
    ax2.plot(digital_values, label='Digital Signal', color='orange')
    ax2.set_title('Digital Values')
    ax2.set_xlabel('Time [ms]')
    ax2.set_ylabel('Digital Value')
    ax2.set_ylim([-0.5, 1.5])
    ax2.legend()
    ax2.grid()

    # Limit the size of the data buffer to avoid memory issues
    if len(analog_values) > 200:
        analog_values.pop(0)
    if len(digital_values) > 200:
        digital_values.pop(0)

'''
thread to manage input requests to user
if the mcu starts sending data again, the input request is
aborted (thread killed)
'''
def request_user_input(ser):
    global input_requested, data_incoming
    try:
        while running:
            if not data_incoming:
                user_input = input()
                if handle_cli_command(user_input):
                    ser.write((user_input).encode('utf-8'))
                else:
                    print("Command not valid.")
            data_incoming = False
    except Exception as e:
        print(f"Error while getting user input: {e}")
    finally:
        input_requested = False

'''
check if the command is a valid one
otherwise, reject
'''
def handle_cli_command(cli_command):
    if cli_command in ["raw", "moving average", "random noise"]:
        return 1
    # unknown command
    return 0

'''
try connect to the serial port.
if it is not possible, try again after RETRY_DELAY time
'''
def connect_serial():
    global ser
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            ser.reset_output_buffer()#clear residual data of buffer
            ser.reset_input_buffer()
            print(f"Connected to {SERIAL_PORT}")
            return ser
        except serial.SerialException as e:
            print(f"Failed to connect to {SERIAL_PORT}, retrying in {RETRY_DELAY} seconds...")
            time.sleep(RETRY_DELAY)


# init serial connection
try:
    ser = connect_serial()
    signal.signal(signal.SIGINT, signal_handler)

    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.set_title('Analog Values')
    ax2.set_title('Digital Values')
    plt.subplots_adjust(hspace=0.4) 
    manager = plt.get_current_fig_manager()
    manager.resize(1000, 900)

    #thread for sounds
    if(PLAY_SOUNDS):
        sound_thread = threading.Thread(target=play_sound)
        sound_thread.daemon = True
        sound_thread.start()

    #thread for serial reading
    serial_thread = threading.Thread(target=read_serial, args=(ser,))
    serial_thread.daemon = True
    serial_thread.start()

    #'thread' for plotting data
    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

    plt.show(block=True)

except Exception as e:
    print("An error has occurred, more details follow:\n", e)

finally:
    running = False
    if ser and not ser.closed:
        ser.close()
    print("Connection closed.")