# A simple console for sending commands directly to the Arduino
import serial
import sys

serial_port = "/dev/ttyACM0"
if len(sys.argv) > 1:
    serial_port = sys.argv[1]
    print "Using serial port " + serial_port

comm = serial.Serial(serial_port, 115200)

while True: 
    print(comm.readline())
    cmd = raw_input("> ")
    
    if cmd == "?":
        while comm.inWaiting() > 0:
            print(comm.readline())
            
    else:
        cmd.rstrip()
        comm.write(cmd + "\r\n")
        comm.flush()