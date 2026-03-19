import serial
import csv
import sys
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# --- SERIAL CONFIG ---
# REMEMBER TO CHANGE THE SERIAL PORT
SERIAL_PORT = '/dev/cu.usbserial-A50285BI' 
BAUD_RATE = 19200
OUTPUT_FILE = 'flight_log.csv'

# Lists to save datas for plots
times = []
altitudes = []
velocities = []
states = []

print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
print("Press CTRL+C to terminate the recording and generate plots.")

try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, \
         open(OUTPUT_FILE, 'w', newline='') as csvfile:
        
        csv_writer = csv.writer(csvfile)
        header_written = False

        while True:
	    # Read a row from serial
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                continue
                
            print(f"Received: {line}")
            
	    # If it is the CSV header of our C++, we write it on the file
            if "Time,Altitude,Velocity,State" in line:
                if not header_written:
                    csv_writer.writerow(["Time", "Altitude", "Velocity", "State"])
                    header_written = True
                continue
            
	    # Try to convert numeric values
            try:
                parts = line.split(',')
                if len(parts) == 4:
                    t = float(parts[0])
                    alt = float(parts[1])
                    vel = float(parts[2])
                    state = int(parts[3])
                    
		    # Save it on python lists
                    times.append(t)
                    altitudes.append(alt)
                    velocities.append(vel)
                    states.append(state)
                    
		    # Save it on the CSV backup 
                    csv_writer.writerow([t, alt, vel, state])
            except ValueError:
		# Malformed row
                pass

except KeyboardInterrupt:
    print("\nRecording stopped by user. Generating plot...")
except Exception as e:
    print(f"\nConnection error: {e}")
    print("Check the serial port name and ensure it is not already open in 'screen'!")
    sys.exit(1)

# ==========================================
# COLOR PLOT GENERATION
# ==========================================
if len(times) > 0:
    plt.figure(figsize=(10, 6))
    plt.title('Rocket Altitude Telemetry', fontsize=16)
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('Relative Altitude (meters)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)

    # Draw colored segments between each pair of points
    for i in range(len(times) - 1):
        x_segment = [times[i], times[i+1]]
        y_segment = [altitudes[i], altitudes[i+1]]
        current_state = states[i]

        # Choose color based on state (IDLE=0, ASCENDING=1, APOGEE=2, DESCENDING=3)
        if current_state == 0:
            color = 'gray'
        elif current_state == 1:
            color = 'blue'
        elif current_state == 2:
            color = 'red'
        elif current_state == 3:
            color = 'darkorange'
        else:
            color = 'black'

        # Increased linewidth for apogee to make it stand out
        linewidth = 4 if current_state == 2 else 2
        
        plt.plot(x_segment, y_segment, color=color, linewidth=linewidth)
        
        # Place a large red star marker if we are at the moment of apogee
        if current_state == 2:
             plt.plot(times[i], altitudes[i], marker='*', color='red', markersize=12)

    # Custom legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='gray', lw=2, label='IDLE (Grounded)'),
        Line2D([0], [0], color='blue', lw=2, label='ASCENDING'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='red', markersize=12, label='APOGEE DETECTED'),
        Line2D([0], [0], color='darkorange', lw=2, label='DESCENDING / RECOVERY')
    ]
    plt.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.show()
else:
    print("No valid data received to create the plot.")
