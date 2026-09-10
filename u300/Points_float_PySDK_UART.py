import serial
import serial.tools.list_ports
import struct


class SerialPortDataManager:
    def __init__(self):
        self.ser = serial.Serial()

    def readAndParseUart(self, ser):
        magicWord = bytearray(b'\x02\x01\x04\x03\x06\x05\x08\x07')
        # Find the start of the frame by locating the magic word
        index = 0
        magicByte = ser.read(1)
        frameData = bytearray(b'')
        while not magicByte:
            magicByte = ser.read(1)
        while True:
            # Found a matching byte
            if magicByte[0] == magicWord[index]:
                index += 1
                frameData.append(magicByte[0])
                if index == 8:  # Found the full magic word
                    break
                magicByte = ser.read(1)

            else:
                if index == 0:  # When you fail, you need to compare your byte against the first byte of the sequence
                    magicByte = ser.read(1)
                index = 0  # Reset index
                frameData = bytearray(b'')  # Reset current frame data

        # Read in the version from the header
        versionBytes = ser.read(4)

        frameData += bytearray(versionBytes)

        # Read in the length from the header
        lengthBytes = ser.read(4)
        frameData += bytearray(lengthBytes)
        frameLength = int.from_bytes(lengthBytes, byteorder='little')

        # Subtract the bytes that have already been read, such as the magic word, version, and length
        # This ensures that we only read the part of the frame that is missing
        frameLength -= 16

        # Read the rest of the frame
        frameData += bytearray(ser.read(frameLength))
        return frameData

    def get_port_list(self):
        """
        Retrieve a list of all COM ports on the current system.
        :return: A list containing the names of all COM ports.
        """
        com_list = []  # A list to save port names
        ports = serial.tools.list_ports.comports()  # Get the local machine ports, returns a list
        for port in ports:
            com_list.append(port.device)  # Save the port to the list
        return com_list  # Return the list

    def select_com_port(self):
        """
        Prompt the user to select a COM port.
        """
        com_list = self.get_port_list()
        if not com_list:
            print("No COM ports found.")
            return None
        print("The available COM ports are:")
        for index, com in enumerate(com_list):
            print(f"{index + 1}. {com}")

        try:
            selection = int(input("Please enter the code in front of the port number according to the prompt above:"))
            if 1 <= selection <= len(com_list):
                return com_list[selection - 1]
            else:
                print("The entered code is invalid, please re-enter.")
                return self.select_com_port()
        except ValueError:
            print("The entered is not a valid number, please re-enter.")
            return self.select_com_port()

    def open_data(self, select_com):
        if not self.ser.is_open:
            com = select_com
            btr = 921600
            try:
                # self.ser = serial.Serial(com, btr, parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=0.3)
                self.ser = serial.Serial(com, btr, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                         timeout=0.08)
                self.ser.reset_output_buffer()

            except:
                print('The serial port does not exist or is occupied.')

            if self.ser.is_open:
                while True:
                    dat = self.readAndParseUart(self.ser)
                    frame = struct.unpack('I', dat[20:24])[0]
                    tlv_num = struct.unpack('I', dat[32:36])[0]
                    print("Current frame：", frame)
                    version = " ".join(f"{byte:02x}" for byte in dat[8:12])
                    print("Version:", version)
                    tlvindex = 40
                    if tlv_num > 0:
                        for i in range(tlv_num):
                            tlv = struct.unpack('I', dat[tlvindex:tlvindex + 4])[0]  # tlv
                            if tlv == 1:
                                tlvlength = struct.unpack('I', dat[tlvindex + 4:tlvindex + 8])[0]
                                datain = dat[tlvindex + 8:tlvindex + 8 + tlvlength]
                                tlvindex = tlvindex + 8 + tlvlength
                                points_num = struct.unpack('I', dat[28:32])[0]
                                print("Number of points targets:{}".format(points_num))
                                for k in range(points_num):
                                    m = k * 16
                                    x = (struct.unpack('f', datain[m:m + 4])[0])
                                    y = (struct.unpack('f', datain[m + 4:m + 8])[0])
                                    z = (struct.unpack('f', datain[m + 8:m + 12])[0])
                                    v = (struct.unpack('f', datain[m + 12:m + 16])[0])
                                    print("points index:{}".format(k + 1))
                                    print("x:{} m".format(x))
                                    print("y:{} m".format(y))
                                    print("z:{} m".format(z))
                                    print("v:{} m/s".format(v))
                                    print("")
                                    print("")

                            else:
                                break
                    else:
                        print("Frame:{}".format(frame))
                        print("Number of points targets:0")

manager = SerialPortDataManager()
selected_port = manager.select_com_port()
if selected_port:
    manager.open_data(selected_port)