# Test how quickly we can send commands
import serial
import sys

comm = serial.Serial("/dev/ttyACM0", 115200)

# wait for READY
comm.readline()

num_pings = 1000

while (num_pings > 0): 
    comm.write("RUN 0.0 0.0 0.0\r\n")
    comm.flush()
    comm.readline()
    num_pings = num_pings - 1
    sys.stdout.write('.')
    sys.stdout.flush()