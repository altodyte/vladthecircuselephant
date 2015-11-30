import sys
import glob
import serial
import os

def run():
    ports = serial_ports()
    print(ports)
    ser = serial.Serial(ports[0],9600,timeout=2)
    # fname = raw_input("Enter a filename to store scan data: ")

    motorNum = 4 # use the 1-4 numbers
    runNum = 1
    
    speedSet = raw_input("Enter speed for the experiment: ")
    pathPrefix = "Data\\"
    motorPrefix = "Motor"+str(motorNum)+"\\"
    runPrefix = "run"+str(runNum)
    path = pathPrefix+motorPrefix+runPrefix
    fname = path+"\\"+speedSet+"step"
    exCode = raw_input("Confirm file location: "+fname+".csv (Y/n) ")
    if not ((exCode.lower()=='y') or (exCode.lower()=='yes') or (exCode=='')):
        print("ABORT MISSION")
        return 1
    if not os.path.exists(path):
        os.makedirs(path)
    f = open(fname+".csv",'w')
    print ser.readline() # "Enter \'+\' and commands to set scan parameters "
    print ser.readline() # "or send any other character to begin."
    # ser.write(raw_input(""))
    ser.write(speedSet)
    print ser.readline() # "Start of Scan: "
    # scan = True
    # while scan:
    for _ in range(1000):
        line = ser.readline().strip()
        print(line)
        f.write(line+'\n')
    f.close()
    ser.write("0")


def serial_ports():
    """Lists serial ports
    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """
    # http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

if __name__=='__main__':
    run()