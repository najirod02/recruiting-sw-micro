import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import signal
import sys
import threading

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

analog_values = []
digital_values = []
running = True
input_requested = False

'''
FIXME:
If we close the plot, the program terminates
'''

def signal_handler(sig, frame):
    global running
    running = False
    print("\nStopping data acquisition...")
    plt.close('all')

def read_serial(ser):
    global running, input_requested
    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(line)
                if input_requested and (line.startswith("A:") or line.startswith("D:")):
                    # cancel input request if transmission resumes
                    input_requested = False
                    print("Input cancelled due to resumed transmission.")

                if line.startswith("A:"):
                    try:
                        analog_value = float(line.split(':')[1])
                        analog_values.append(analog_value)
                    except ValueError:
                        continue

                elif line.startswith("D:"):
                    try:
                        digital_value = int(line.split(':')[1])
                        digital_values.append(digital_value)
                    except ValueError:
                        continue
                
                elif line.startswith("C:"):
                    if not input_requested:
                        input_requested = True
                        threading.Thread(target=request_user_input, args=(ser,)).start()

        except Exception as e:
            if not running:
                break
            print("Error reading serial data:", e)

def request_user_input(ser):
    global input_requested
    try:
        user_input = input()
        if input_requested:
            ser.write((user_input).encode('utf-8'))
    except Exception as e:
        print(f"Error while getting user input: {e}")
    finally:
        input_requested = False

def update_plot(frame):
    ax1.clear()
    ax2.clear()

    # update analog plot
    ax1.plot(analog_values, label='Analog Signal', color='blue')
    ax1.set_title('Analog Values')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Analog Value')
    ax1.set_ylim([0, 5000])
    ax1.legend()
    ax1.grid()

    # update digital plot
    ax2.plot(digital_values, label='Digital Signal', color='orange')
    ax2.set_title('Digital Values')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Digital Value')
    ax2.set_ylim([-1, 2])
    ax2.legend()
    ax2.grid()

    # delete old values to avoid memory issues
    if len(analog_values) > 200:
        analog_values.pop(0)
    if len(digital_values) > 200:
        digital_values.pop(0)

# init serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    signal.signal(signal.SIGINT, signal_handler)

    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.set_title('Analog Values')
    ax2.set_title('Digital Values')

    serial_thread = threading.Thread(target=read_serial, args=(ser,))
    serial_thread.daemon = True
    serial_thread.start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100)

    # the plot should not steal the focus from other windows
    plt.show(block=True)


except Exception as e:
    print("An error has occurred, more details follow:\n", e)

finally:
    running = False
    ser.close()
    print("Connection closed.")
    sys.exit(0)
