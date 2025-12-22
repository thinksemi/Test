import serial
import time
import os


CHUNK_SIZE = 256
BIN_FILE = "Demo2_CMSIS.bin"
chunk_number = 0
# Open serial port
ser = serial.Serial(
    port='COM7',      # change this
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

time.sleep(2)  # wait for port to stabilize

# Send data
message = "Hello STM32\r\n"
ser.write(message.encode('utf-8'))

print("Data sent")


if(os.path.exists(BIN_FILE)):
    print("File exists")
else:
    print("File does not exist")


file_size = os.path.getsize(BIN_FILE)
print(f"Size of Helloworld.py: {file_size} bytes")  


ser.write('a'.encode('utf-8'))  # signal to start sending file
ser.write('Bootloader Started\r\n'.encode('utf-8'))

while ser.read(19) != b'Bootloader Started\n':
    # wait for bootloader to be ready
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        
        print(data.decode(errors='ignore'))
    pass

with open(BIN_FILE, 'rb') as f:
    while True:
        chunk = f.read(CHUNK_SIZE)
        if not chunk:
            break

        if ser.read(15) == b'Message Written\n':
            ser.write(chunk)



        ser.write(chunk)
        time.sleep(0.1)  # small delay to avoid overwhelming the receiver
        print(f"Sent chunk {chunk_number} -> {len(chunk)} bytes")
        chunk_number += 1

ser.close()
