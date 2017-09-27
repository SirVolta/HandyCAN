import serial


ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.25)

ser.reset_input_buffer()
ser.reset_output_buffer()

output = []
outputReady = False
alive = True


data = []
previousByte = 0


def handyCANDecode(message):
    if message[0] != 0xAA or message[1] != 0xAB:
        print("not a valid handycan message: invalid header")
        return
    if message[-1] != 0xBB or message[-2] != 0xBA:
        print("not a valid handycan message: invalid footer")
        return

    StdId = message[2] | message[3] << 8;

    out = {}
    out['source'] = (StdId & 0x3E0) >> 5;
    out['dest'] = StdId & 0x1F
    out['length'] = message[4]
    out['data'] = []
    for i in range(5, out['length'] + 5, 1):
        out['data'].append(message[i])

    return out


def printHandyCANPackage(package):
    prettyOutput = "Incoming CAN packet from {} ({}) to {} ({}) with length {}\nContaining data {} {}".format(
    package['source'], hex(package['source']),
               package['dest'], hex(package['dest']),
               package['length'], package['data'], [hex(elem) for elem in package['data']])
    print(prettyOutput)
    
    
    
        
testdata = []

while alive:
    indata = ser.read(1)
    if indata:
        if indata == b'\xab':
            if previousByte ==  b'\xaa':
                print("clear")
                data = [int.from_bytes(previousByte, byteorder='big')]

        if indata == b'\xbb':
            if previousByte == b'\xba':
                output = data
                data.append(int.from_bytes(indata, byteorder='big'))
                hcp = handyCANDecode(data)
                printHandyCANPackage(hcp)
                
        data.append(int.from_bytes(indata, byteorder='big'))
        previousByte = indata
