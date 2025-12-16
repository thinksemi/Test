import serial
import time

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

ser.close()
