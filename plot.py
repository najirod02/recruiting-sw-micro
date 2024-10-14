import serial
import matplotlib.pyplot as plt
import signal
import sys
import threading

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

analog_values = []
digital_values = []
running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\nConnection closed.")
    plt.close('all')
    sys.exit(0)

def read_serial(ser):
    global running
    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(line)
                if line.startswith("A:"):
                    # Analog value
                    try:
                        analog_value = float(line.split(':')[1])
                        analog_values.append(analog_value)
                    except ValueError:
                        continue

                elif line.startswith("D:"):
                    # Digital value
                    try:
                        digital_value = int(line.split(':')[1])
                        digital_values.append(digital_value)
                    except ValueError:
                        continue
        except Exception as e:
            if not running:
                break  # just exit as the port is closed
            print("Error reading serial data:", e)

# serial connection initialization
try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        signal.signal(signal.SIGINT, signal_handler)
        
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1)
        ax1.set_title('Analog Values')
        ax2.set_title('Digital Values')

        serial_thread = threading.Thread(target=read_serial, args=(ser,))
        serial_thread.start()

        while running:
            ax1.clear()
            ax2.clear()

            # Update the analog plot
            ax1.plot(analog_values, label='Analog Signal', color='blue')
            ax1.set_title('Analog Values')
            ax1.set_xlabel('Time')
            ax1.set_ylabel('Analog Value')
            ax1.legend()
            ax1.grid()

            # Update the digital plot
            ax2.plot(digital_values, label='Digital Signal', color='orange')
            ax2.set_title('Digital Values')
            ax2.set_xlabel('Time')
            ax2.set_ylabel('Digital Value')
            ax2.set_ylim([-1, 2])
            ax2.legend()
            ax2.grid()

            plt.pause(0.1)

            if len(analog_values) > 200:
                analog_values.pop(0)
            if len(digital_values) > 200:
                digital_values.pop(0)

except Exception as e:
    print("An error has occurred, more details follow:\n", e)
