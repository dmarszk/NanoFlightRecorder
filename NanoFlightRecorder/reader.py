import sys
import struct


MEAS_TYPE_START = 0
MEAS_TYPE_AHRS = 1
MEAS_TYPE_ACCELERATION = 2
MEAS_TYPE_GYRO = 3
MEAS_TYPE_MAG = 4
MEAS_TYPE_BAROM = 5

# Default OBSW version (buggy)
obswVersion = 1

class Reader:
    def __init__(self, inputFile, outputFile):
        self.inputFile = inputFile
        self.outputFile = outputFile
        self.lastTimestamp = 0
        self.timestampShift = 0
    def handleVer1Measurement(self):
        unpacked = struct.unpack('<Ii', inputFile.read(8))
        pressure = unpacked[0]
        temp = unpacked[1]
        outputFile.write('{0} {1} {2} \n'.format(self.timestamp, pressure, temp))
    # Version 1 has only 1 measurement type due to a bug
    dataDispatcher = {
        MEAS_TYPE_START : handleVer1Measurement
    }
    def process(self):
        while True:
            headerBytes = inputFile.read(4)
            if not headerBytes:
                break
            header = struct.unpack('<I', headerBytes)[0]
            measType = header >> 28
            self.timestamp = (header & 0x0FFFFFFF) + self.timestampShift
            if self.timestamp < self.lastTimestamp:
                self.timestampShift += 0x0FFFFFFF
                self.timestamp += 0x0FFFFFFF
            self.dataDispatcher[measType](self)
            self.lastTimestamp = self.timestamp


inputFile = open(sys.argv[1], 'rb')
outputFile = open(sys.argv[1] + ".csv", 'wb')
reader = Reader(inputFile, outputFile)
reader.process()
outputFile.close()
inputFile.close()
