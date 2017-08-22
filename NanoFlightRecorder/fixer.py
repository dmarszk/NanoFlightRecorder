import sys

inputFile = open(sys.argv[1], 'rb')
outputFile = open(sys.argv[1] + "_FIXED", 'wb')
while True:
    inBytes = inputFile.read(77)
    if not inBytes:
        break
    outputFile.write(inBytes[:12])
outputFile.close()
inputFile.close()
