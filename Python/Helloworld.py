import serial
import time
import os


CHUNK_SIZE = 256
BIN_FILE = "Helloworld.py"
chunk_number = 0
# Open serial port
ser = serial.Serial(
    port='COM4',      # change this
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


if(os.path.exists('Helloworld.py')):
    print("File exists")
else:
    print("File does not exist")


file_size = os.path.getsize('Helloworld.py')
print(f"Size of Helloworld.py: {file_size} bytes")  


with open(BIN_FILE, 'rb') as f:
    while True:
        chunk = f.read(CHUNK_SIZE)
        if not chunk:
            break
        ser.write(chunk)
        time.sleep(0.1)  # small delay to avoid overwhelming the receiver
        print(f"Sent chunk {chunk_number} -> {len(chunk)} bytes")
        chunk_number += 1

ser.close()
