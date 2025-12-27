import serial
import time
import os
import math

PORT = "COM4"        # Change to your port
BAUDRATE = 9600
TIMEOUT = 1.0        # seconds

CHUNK_SIZE = 16
# BIN_FILE = "Demo2_CMSIS.bin"
# BIN_FILE = "TestV1_Blink.bin"
# BIN_FILE = "Test.txt"
BIN_FILE = "Digital_signature/signed_app.bin"  # Digitally signed binary file
chunk_number = 0
file_size=0
Tx_chunk_length=0

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

    transmit_file()
    Tx_chunk_length= math.ceil(file_size/CHUNK_SIZE)
    print(f"Total Chunks to transmit: {Tx_chunk_length}")
    start_command = b'start' + str(Tx_chunk_length).zfill(5).encode() + b'\r\n'
    print(f"Sending command: {start_command}")
    # ser.write(start_command)


    print("Entering interactive mode. Type messages to send. Type 'Exit' to quit.")  

    while True:
            if ser.in_waiting:
                                            #data = ser.read(ser.in_waiting)
                                            data = ser.readline()
                                            print(data.decode(errors='ignore'))
                                            if(data == b'Ready from STM32\r\n'):
                                                print("Iam here...") 
                                                time.sleep(0.2) 
                                                # ser.write(b"start00002")
                                                ser.write(bytes("start"+ str(Tx_chunk_length).zfill(5), 'utf-8'))
                                                ser.flush()
                                                while ser.in_waiting == 0:
                                                    pass
                                                time.sleep(0.2)  # small delay to avoid overwhelming the receiver
                                                data = ser.readline()
                                                print(data.decode(errors='ignore'))
                                                while data != b'Boot\r\n':
                                                    data = ser.readline()
                                                    print(data.decode(errors='ignore'))
                                                    time.sleep(0.2)  # small delay to avoid overwhelming the receiver
                                                    ser.reset_input_buffer()
                                                    ser.reset_output_buffer()
                                                    print("Boot command received...")
                                                break
    # while True:
    #      if ser.in_waiting:
    #                                     #data = ser.read(ser.in_waiting)
    #                                     data = ser.readline()
    #                                     print(data.decode(errors='ignore'))
    #                                     if(data == b'Ready from STM32\r\n'):
    #                                         print("Iam here...") 
    #                                         time.sleep(0.2) 
    #                                         ser.write(b"start00001")
    #                                         ser.flush()
    #                                         while ser.in_waiting == 0:
    #                                                pass
    #                                         time.sleep(0.2)  # small delay to avoid overwhelming the receiver
    #                                         data = ser.readline()
    #                                         print(data.decode(errors='ignore'))
    #                                         ser.write(b'1111111111111111\r\n')
    #                                         data = ser.readline()
    #                                         print(data.decode(errors='ignore'))
    #                                         time.sleep(5)  # small delay to avoid overwhelming the receiver
    #                                         break


    # ser.reset_input_buffer()
    # ser.reset_output_buffer()
    with open(BIN_FILE, 'rb') as f:

        while True:
                    # chunk = f.read(CHUNK_SIZE)
                    # if not chunk:
                    #     break
                    # msg = input("Enter message: ")
                    # # time.sleep(0.1)
                    # ser.write(msg.encode() + b'\r\n')
                    # ser.write(b'start00003\r\n')
                    # data = ser.readline()
                    # print(data.decode(errors='ignore'))
                    # time.sleep(0.2)  # small delay to avoid overwhelming the receiver
                    
                    # data = ser.readline()
                    # print(data.decode(errors='ignore'))
                    # time.sleep(0.2)  # small delay to avoid overwhelming the receiver
                    # while data != b'Boot\r\n':
                    #     data = ser.readline()
                    #     print(data.decode(errors='ignore'))
                    #     time.sleep(0.2)  # small delay to avoid overwhelming the receiver
                    #     ser.reset_input_buffer()
                    #     ser.reset_output_buffer()
                    #     print("Boot command received...")

                    if (ser.in_waiting or data==b'Boot\r\n'):
                        
                        if(data ==""):
                              data = ser.readline()
                              print(data.decode(errors='ignore'))
                        if(data == b'Boot\r\n'):
                            print("Transmit data...")
                            data=""
                            # transmit_file()
                            # ser.reset_input_buffer()
                            # ser.reset_output_buffer()
                            chunk = f.read(CHUNK_SIZE)
                            chunk_number += 1
                            # if(len(chunk)<16): 
                            if(chunk_number>=Tx_chunk_length and len(chunk)<16):
                                print("Chunk is appending...")
                                print(len(chunk))
                                chunk = bytearray(chunk)
                                if(len(chunk)<16):
                                    chunk.extend([0xFF] * (16 - len(chunk)))
                                chunk = bytes(chunk)
                                print(len(chunk))
                                
                            ser.write(chunk)
                            time.sleep(1)  # small delay to avoid overwhelming the receiver
                            print(f"Sent chunk {chunk_number} -> {len(chunk)} bytes")
                            # chunk_number += 1
                            # ser.reset_input_buffer()
                            # ser.reset_output_buffer()
                        elif(data == b'Exit\r\n'):
                            # ser.reset_input_buffer()
                            # ser.reset_output_buffer()
                            break
                        data = ser.readline()
                        print(data.decode(errors='ignore'))
                        #break
                    else:
                        data = ""
                        # break
    print("Exiting...")
    ser.close()



def transmit_file():

    if(os.path.exists(BIN_FILE)):
        print("File exists")
        print("Transmitting file...")

        global file_size, Tx_chunk_length
        file_size = os.path.getsize(BIN_FILE)
        print(f"Size of {BIN_FILE}: {file_size} bytes") 

        # Tx_chunk_length= file_size/CHUNK_SIZE
        # print(f"Total Chunks to transmit: {Tx_chunk_length}") 
    else:
        print("File does not exist")
    
    # pass  # Placeholder for file transmission logic

if __name__ == "__main__":
   TXRxFile_inChunks_test()
