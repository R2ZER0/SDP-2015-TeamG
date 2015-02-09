# A simple console for sending commands directly to the Arduino
import serial
import sys

serial_port = "/dev/ttyACM0"
if len(sys.argv) > 1:
    serial_port = sys.argv[1]
    print "Using serial port " + serial_port

comm = serial.Serial(serial_port, 115200)

valid_commands = ["PING", "LED", "RUN", "KICK", "CATCH", "MPU"]

# Wait for STARTED
print("Waiting for STARTED, could take 10-15 seconds...")
while True:
    line = comm.readline().rstrip()
    
    print(line)
    if(line == "STARTED"):
        break

while True: 
    print("")
    cmd = raw_input("> ").rstrip().upper();

    if(cmd == "EXIT"):
        exit()
        
    if(cmd.split(" ",1)[0] not in valid_commands):
        print("Unknown command")
        continue

    comm.write(cmd + "\r\n")
    comm.flush()
    
    # Wait for DONE or FAILED
    while True:
        line = comm.readline().rstrip()
        
        print(line)
        if(line == "DONE" or line == "FAILED"):
            break

