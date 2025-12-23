import serial
import time
import os

PORT = "COM4"        # Change to your port
BAUDRATE = 9600
TIMEOUT = 1.0        # seconds

CHUNK_SIZE = 256
BIN_FILE = "Demo2_CMSIS.bin"
chunk_number = 0

TEST_DATA = b"HELLO_STM32_123\r\n"

def TXRxFile_inChunks_test():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT
    )

    time.sleep(2)  # Allow MCU reset
    chunk_number = 0

    print(f"TX ({len(TEST_DATA)} bytes): {TEST_DATA}")
    ser.write(TEST_DATA)
    ser.flush()

    rx = ser.read(len(TEST_DATA))

    print(f"RX ({len(rx)} bytes): {rx}")

    if rx == TEST_DATA:
            print("✅ Loopback SUCCESS")
    else:
            print("❌ Loopback FAILED")

    print("Entering interactive mode. Type messages to send. Type 'Exit' to quit.")    

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    with open(BIN_FILE, 'rb') as f:

        while True:
                    chunk = f.read(CHUNK_SIZE)
                    # if not chunk:
                    #     break
                    msg = input("Enter message: ")
                    # time.sleep(0.1)
                    ser.write(msg.encode() + b'\r\n')
                    time.sleep(1)  # small delay to avoid overwhelming the receiver

                    if ser.in_waiting:
                        #data = ser.read(ser.in_waiting)
                        data = ser.readline()
                        print(data.decode(errors='ignore'))
                        if(data == b'Boot\r\n'):
                            print("Transmit data...")
                            # transmit_file()
                            # ser.reset_input_buffer()
                            # ser.reset_output_buffer()
                            ser.write(chunk)
                            time.sleep(1)  # small delay to avoid overwhelming the receiver
                            print(f"Sent chunk {chunk_number} -> {len(chunk)} bytes")
                            chunk_number += 1
                            ser.reset_input_buffer()
                            ser.reset_output_buffer()
                        elif(data == b'Exit\r\n'):
                            # ser.reset_input_buffer()
                            # ser.reset_output_buffer()
                            break
                        #break
    print("Exiting...")
    ser.close()



def transmit_file():

    if(os.path.exists(BIN_FILE)):
        print("File exists")
        print("Transmitting file...")

        file_size = os.path.getsize(BIN_FILE)
        print(f"Size of {BIN_FILE}: {file_size} bytes")  
    else:
        print("File does not exist")
    
    # pass  # Placeholder for file transmission logic

if __name__ == "__main__":
   TXRxFile_inChunks_test()
