import serial
import time
import numpy as np

# Serial COM constants
DATACOMPORT = 'COM8' # '/dev/tty.usbmodemfa141'
CONFIGCOMPORT = 'COM7'
DATABAUD = 921600
CONFIGBAUD = 115200
CONFIGFILE = './ISK_6m_default.cfg'

# TLV Types
POINT_CLOUD = 1020
TARGET_OBJECT_LIST = 1010
TARGET_INDEX = 1011
TARGET_HEIGHT = 1012
PRESENCE_INDICATION = 1021

# Initialize serial port
def init_serial(com_port, baud_rate):
    print("Func: init_sys_serial")
    # serialh = 0
    try:
        serialh = serial.Serial(com_port, baud_rate, bytesize = 8, parity = serial.PARITY_NONE, stopbits = 1, timeout = 0.01)
        serialh.reset_input_buffer()
    except:
        print("Failed to open serial port! ", com_port)
    return serialh


# Close serial port
def deinit_serial(serialh):
    print("Func: deinit_serial")
    serialh.close()


# Read config file and send it to CONFIGPORT
def send_config_data(serialh):
    print("Func: send_config_data")

    with open(CONFIGFILE) as f:
        lines = [line for line in f]

    print("Sending config data to config port...\n\n")
    for line in lines:
        serialh.write(line.encode())
        time.sleep(0.1)
        print(line)


def stream_data(serialh):
    print("Func: stream_data")

    try: 
        while True:
            byteBuffer = np.zeros(2**15,dtype = 'uint8')
            byteBufferLength = 0

            # Constants
            OBJ_STRUCT_SIZE_BYTES = 12
            BYTE_VEC_ACC_MAX_SIZE = 2**15
            maxBufferSize = 2**15
            tlvHeaderLengthInBytes = 8
            poinUnitInBytes = 20
            pointLengthInBytes = 8
            magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
            
            # Initialize variables
            magicOK = 0 # Checks if magic number has been read
            dataOK = 0 # Checks if the data has been read correctly
            targetDetected = 0 # Checks if a person has been detected
            frameNumber = 0
            targetObj = {}
            pointObj = {}
            
            readBuffer = serialh.read(serialh.in_waiting)
            # HEX_STRING = "0201040306050807" + "01020000" + "28000000" + "03040806" + "01000000" + "02000000" + "03000000" + "04000000" + "05000000" \
            #             + "FC030000" + "44000000" + "CDCCCC3D" + "CDCCCC3D" + "CDCCCC3D" + "CDCCCC3D" + "CDCCCC3D" \
            #             + "0A" + "32" + "0F00" + "0E00" + "0D00"
                        
            # size is 8 bytes header, 20 bytes unit, 8 bytes per point (5 points) 
            # readBuffer = bytearray.fromhex(HEX_STRING)
            byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
            byteCount = len(byteVec)
            
            # Check that the buffer is not full, and then add the data to the buffer
            if (byteBufferLength + byteCount) < maxBufferSize:
                byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
                byteBufferLength = byteBufferLength + byteCount
                
            # Check that the buffer has some data
            if byteBufferLength > 16:    
                # Check for all possible locations of the magic word
                possibleLocs = np.where(byteBuffer == magicWord[0])[0]
            
                # Confirm that is the beginning of the magic word and store the index in startIdx
                startIdx = []
                for loc in possibleLocs:
                    check = byteBuffer[loc:loc + 8]
                    if np.all(check == magicWord):
                        startIdx.append(loc)
            
                # Check that startIdx is not empty
                if startIdx:
            
                    # Remove the data before the first start index
                    if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                        byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                        byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                        byteBufferLength = byteBufferLength - startIdx[0]
            
                    # Check that there have no errors with the byte buffer length
                    if byteBufferLength < 0:
                        byteBufferLength = 0
            
                    # word array to convert 4 bytes to a 32 bit number
                    word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
            
                    # Read the total packet length
                    totalPacketLen = np.matmul(byteBuffer[12:12 + 4], word)
            
                    # Check that all the packet has been read
                    if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                        magicOK = 1
            
            print("\n\n====================================\n\n")\
            
            # If magicOK is equal to 1 then process the message
            if magicOK:
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
            
                # Initialize the pointer index
                idX = 0
            
                # Read the header
                # Read the header
                magicNumber = byteBuffer[idX:idX + 8]
                idX += 8
                version = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
                idX += 4
                totalPacketLen = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                platform = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
                idX += 4
                frameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                timeStamp = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                numDetectedObj = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                numTLVs = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                subFrameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                
                print("Magic Number: ", magicNumber)
                print("Version: ", version)
                print("Total Packet Len: ", totalPacketLen)
                print("Platform: ", platform)
                print("Frame Number: ", frameNumber)
                print("Timestamp: ", timeStamp)
                print("Number of Detected Object: ", numDetectedObj)
                print("Num of TLVs: ", numTLVs)
                print("Sub Frame Number: ", subFrameNumber)
            
                # Read the TLV messages
                for tlvIdx in range(numTLVs):
                # word array to convert 4 bytes to a 32 bit number
                    word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
                    
                    # Initialize the tlv type
                    tlv_type = 0
            
                    try: 
                        # Check the header of the TLV message
                        tlv_type = np.matmul(byteBuffer[idX:idX + 4], word)
                        idX += 4
                        tlv_length = np.matmul(byteBuffer[idX:idX + 4], word)
                        idX += 4

                        print("TLV Num: ", tlv_type)
                        print("TLV Len: ", tlv_length)
                    except:
                        pass
            
                    # Read the data depending on the TLV message
                    if tlv_type == POINT_CLOUD:
                        print("\n\nTLV type: POINT_CLOUD")

                        # word array to convert 4 bytes to a 16 bit number
                        word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                        # Initialize the arrays
                        elevationUnit = np.zeros(1, dtype=np.float32)
                        azimuthUnit = np.zeros(1, dtype=np.float32)
                        dopplerUnit = np.zeros(1, dtype=np.float32)
                        rangeUnit = np.zeros(1, dtype=np.float32)
                        snrUnit = np.zeros(1, dtype=np.float32)

                        # Read the Point unit for this TLV
                        elevationUnit = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]
                        idX += 4
                        azimuthUnit = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]
                        idX += 4
                        dopplerUnit = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]
                        idX += 4
                        rangeUnit = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]
                        idX += 4
                        snrUnit = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]
                        idX += 4
                        
                        print("Elevation Unit: ", elevationUnit)
                        print("Azimuth Unit: ", azimuthUnit)
                        print("Doppler Unit: ", dopplerUnit)
                        print("Range Unit: ", rangeUnit)
                        print("SNR Unit: ", snrUnit)

                        # Calculate the number of detected points
                        numInputPoints = (tlv_length - tlvHeaderLengthInBytes - poinUnitInBytes) // pointLengthInBytes
                        print("Number of Points: ", numInputPoints)

                        # Initialize the arrays
                        elevationVal = np.zeros(numInputPoints, dtype=np.int8)
                        azimuthVal = np.zeros(numInputPoints, dtype=np.int8)
                        dopplerVal = np.zeros(numInputPoints, dtype=np.int16)
                        rangeVal = np.zeros(numInputPoints, dtype=np.uint16)
                        snrVal = np.zeros(numInputPoints, dtype=np.uint16)
            
                        for objectNum in range(numInputPoints):
                            print ("\n\nPoint ", objectNum)
                            # Read the data for each object
                            elevationVal[objectNum] = byteBuffer[idX:idX + 1].view(dtype=np.int8) #* elevationUnit
                            idX += 1
                            azimuthVal[objectNum] = byteBuffer[idX:idX + 1].view(dtype=np.int8) #* azimuthUnit
                            idX += 1
                            dopplerVal[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.int16) #* dopplerUnit
                            idX += 2
                            rangeVal[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.uint16) #* rangeUnit
                            idX += 2
                            snrVal[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.uint16) #* snrUnit
                            idX += 2

                            print("Elevation: ", elevationVal[objectNum])
                            print("Azimuth: ", azimuthVal[objectNum] )
                            print("Doppler Val: ", dopplerVal[objectNum] )
                            print("Range Val: ", rangeVal[objectNum] )
                            print("SNR: ", snrVal[objectNum] )
            
                    elif tlv_type == TARGET_OBJECT_LIST:
                        print("TLV type: TARGET_OBJECT_LIST")
            
                    elif tlv_type == TARGET_INDEX:
                        print("TLV type: TARGET_OBJECT_LIST")

                    elif tlv_type == PRESENCE_INDICATION:
                        print("TLV type: PRESENCE_INDICATION")

                    elif tlv_type == TARGET_HEIGHT:
                        print("TLV type: TARGET_HEIGHT")
            
            
                # Remove already processed data
                if idX > 0:
                    shiftSize = totalPacketLen
                    byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
                    byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
                    byteBufferLength = byteBufferLength - shiftSize
            
                    # Check that there are no errors with the buffer length
                    if byteBufferLength < 0:
                        byteBufferLength = 0

    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    print('\n=======\npointcloud_parser.py\n=======\n')

    # Set config data
    serialh = init_serial(CONFIGCOMPORT, CONFIGBAUD)
    send_config_data(serialh)
    deinit_serial(serialh)

    # Stream point cloud data
    serialh = init_serial(DATACOMPORT, DATABAUD)
    stream_data(serialh) # enters infinite loop that can be quit by keyboard interrupt
    deinit_serial(serialh)
    