# A simple console for sending commands directly to the Arduino
import serial

comm = serial.Serial("/dev/ttyACM0", 115200)

while True: 
    print(comm.readline())
    cmd = raw_input("> ")
    cmd.rstrip()
    comm.write(cmd + "\r\n")
    comm.flush()