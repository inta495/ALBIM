#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2014 Aldux.net"

__license__ = "GPL"
__version__ = "1.5"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import serial, time, struct


class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254

    RC_COMMAND = 150
    RC_PI = 151
    ALTITUDE_ONLY = 153
    SET_ALTITUDE = 152

    """Class initialization"""
    def __init__(self, serPort):

        """Global variables of data"""
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'AUX1':0,'AUX2':0,'AUX3':0,'AUX4':0,'elapsed':0,'timestamp':0}
        self.rcCommand = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.altitude = {'altitude':0,'velocity':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.status = {'cycleTime':0 , 'i2c_errors_count':0 , 'sensor':0 , 'flag':0 , 'global_config':0}
        self.rcPi = {'Ch1':0,'Ch2':0,'Ch3':0,'Ch4':0}
        self.debug = {'ch1':0,'ch2':0,'ch3':0,'ch4':0}
        self.temp = ();
        self.temp2 = ();
        self.temp3 = ();
        self.temp4 = ();
        self.temp5 = ();
        self.temp6 = ();
        self.temp7 = ();
        self.temp8 = ();
        self.temp9 = ();
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0.003
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        """Time to wait until the board becomes operational"""
        wakeup = 10
        try:
            self.ser.open()
            if self.PRINT:
                print "Waking up board on "+self.ser.port+"..."
            for i in range(1,wakeup):
                if self.PRINT:
                    print wakeup-i
                    time.sleep(1)
                else:
                    time.sleep(1)            
        except Exception, error:
            print "\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n"

    """Function for sending a command to the board"""
    def sendCMD(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ ord(i)
        total_data.append(checksum)
        try:
            b = None
            b = self.ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
        except Exception, error:
            #print "\n\nError in sendCMD."
            #print "("+str(error)+")\n\n"
            pass

    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000]
            self.sendCMD(8,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000]
            self.sendCMD(8,MultiWii.SET_RAW_RC,data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    """Function to receive a data packet from the board"""
    def getData(self, cmd):
        self.ser.flushInput()
        self.ser.flushOutput()
        try:
            start = time.time()
            self.sendCMD(0,cmd,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break                        
            if cmd == MultiWii.ATTITUDE:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                self.temp = struct.unpack('<'+'h'*(datalength/2),data)
                #print self.temp debug
                self.ser.flushInput()
                self.ser.flushOutput()
                self.attitude['angx']=float(self.temp[0]/10.0)
                self.attitude['angy']=float(self.temp[1]/10.0)
                self.attitude['heading']=float(self.temp[2])
                self.attitude['elapsed']=round(elapsed,3)
                self.attitude['timestamp']="%0.2f" % (time.time(),) 
                return self.attitude
            if cmd == MultiWii.ALTITUDE:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                self.temp2 = struct.unpack('<'+'ih',data)
                self.ser.flushInput()
                self.ser.flushOutput()
                self.altitude['altitude']=float(self.temp2[0]/100.0)
                self.altitude['velocity']=float(self.temp2[1])
                self.altitude['elapsed']=round(elapsed,3)
                self.altitude['timestamp']="%0.2f" % (time.time(),) 
                return self.altitude    
            elif cmd == MultiWii.RC:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                #print datalength
                self.temp3 = struct.unpack('<'+'h'*(datalength/2),data)
                #print self.temp3
                self.ser.flushInput()
                self.ser.flushOutput()
                self.rcChannels['roll']=self.temp3[0]
                self.rcChannels['pitch']=self.temp3[1]
                self.rcChannels['yaw']=self.temp3[2]
                self.rcChannels['throttle']=self.temp3[3]                
                self.rcChannels['AUX1']=self.temp3[4]
                self.rcChannels['AUX2']=self.temp3[5]
                self.rcChannels['AUX3']=self.temp3[6]
                self.rcChannels['AUX4']=self.temp3[7]
                self.rcChannels['elapsed']=round(elapsed,3)
                self.rcChannels['timestamp']="%0.2f" % (time.time(),)                
                return self.rcChannels
            elif cmd == MultiWii.RC_COMMAND:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                #print "RC_COMMAND"
                self.temp4 = struct.unpack('<'+'h'*(datalength/2),data)                
                self.ser.flushInput()
                self.ser.flushOutput()
                self.rcCommand['roll']=self.temp4[0]
                self.rcCommand['pitch']=self.temp4[1]
                self.rcCommand['yaw']=self.temp4[2]
                self.rcCommand['throttle']=self.temp4[3]
                self.rcCommand['elapsed']=round(elapsed,3)
                self.rcCommand['timestamp']="%0.2f" % (time.time(),)
                return self.rcCommand
            elif cmd == MultiWii.RC_PI:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                #print "RC_COMMAND"
                self.temp5 = struct.unpack('<'+'h'*(datalength/2),data)                
                self.ser.flushInput()
                self.ser.flushOutput()
                self.rcPi['Ch1']=self.temp5[0]
                self.rcPi['Ch2']=self.temp5[1]
                self.rcPi['Ch3']=self.temp5[2]
                self.rcPi['Ch4']=self.temp5[3]
                return self.rcPi
            elif cmd == MultiWii.RAW_IMU:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                self.temp6 = struct.unpack('<'+'h'*(datalength/2),data)
                self.ser.flushInput()
                self.ser.flushOutput()
                self.rawIMU['ax']=float(self.temp6[0])
                self.rawIMU['ay']=float(self.temp6[1])
                self.rawIMU['az']=float(self.temp6[2])
                self.rawIMU['gx']=float(self.temp6[3])
                self.rawIMU['gy']=float(self.temp6[4])
                self.rawIMU['gz']=float(self.temp6[5])
                self.rawIMU['elapsed']=round(elapsed,3)
                self.rawIMU['timestamp']="%0.2f" % (time.time(),)
                return self.rawIMU
            elif cmd == MultiWii.MOTOR:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                self.temp7 = struct.unpack('<'+'h'*(datalength/2),data)
                self.ser.flushInput()
                self.ser.flushOutput()
                self.motor['m1']=self.temp7[0]
                self.motor['m2']=self.temp7[1]
                self.motor['m3']=self.temp7[2]
                self.motor['m4']=self.temp7[3]
                self.motor['elapsed']="%0.3f" % (elapsed,)
                self.motor['timestamp']="%0.2f" % (time.time(),)
                return self.motor
            elif cmd == MultiWii.STATUS:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                #binary                
                #DEBUG
                #print datalength
                # datalength = 11
                # cycleTime              , UINT16 = 2Byte
                # i2c_errors_count       , UINT16 = 2Byte
                # sensor                 , UINT16 = 2Byte
                # flag                   , UINT32 = 4Byte
                # global_conf.currentSet , UINT8  = 1Byte            
                self.temp8 = struct.unpack('<'+'HHH'+'I'+'B',data)           
                self.ser.flushInput()
                self.ser.flushOutput()               
                #binary                
                #self.status = {'cycleTime':0 , 'i2c_errors_count':0 , 'sensor':0 , 'flag':0 , 'global_config':0}
                self.status['cycleTime'] = self.temp8[0]
                self.status['i2c_errors_count'] = self.temp8[1]
                self.status['sensor'] = self.temp8[2]
                self.status['flag'] = self.temp8[3]
                self.status['timestamp']="%0.2f" % (time.time(),)
                return self.status
            elif cmd == MultiWii.DEBUG:
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                checksum = self.ser.read(1)
                elapsed = time.time() - start
                self.temp9 = struct.unpack('<'+'h'*(datalength/2),data)
                self.ser.flushInput()
                self.ser.flushOutput()
                self.debug['ch1']=self.temp9[0]
                self.debug['ch2']=self.temp9[1]
                self.debug['ch3']=self.temp9[2]
                self.debug['ch4']=self.temp9[3]                
                return self.motor
            else:
                return "No return error!"
        except Exception, error:
            #print error
            pass

    """Function to ask for 2 fixed cmds, attitude and rc channels, and receive them. Note: is a bit slower than others"""
    def getData2cmd(self, cmd):
        try:
            start = time.time()
            self.sendCMD(0,self.ATTITUDE,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            self.temp = struct.unpack('<'+'h'*(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()

            self.sendCMD(0,self.RC,[])
            while True:
                header = self.ser.read()
                if header == '$':
                    header = header+self.ser.read(2)
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            self.temp2 = struct.unpack('<'+'h'*(datalength/2),data)
            elapsed = time.time() - start
            self.ser.flushInput()
            self.ser.flushOutput()

            if cmd == MultiWii.ATTITUDE:
                self.message['angx']=float(self.temp[0]/10.0)
                self.message['angy']=float(self.temp[1]/10.0)
                self.message['heading']=float(self.temp[2])
                self.message['roll']=self.temp2[0]
                self.message['pitch']=self.temp2[1]
                self.message['yaw']=self.temp2[2]
                self.message['throttle']=self.temp2[3]
                self.message['elapsed']=round(elapsed,3)
                self.message['timestamp']="%0.2f" % (time.time(),) 
                return self.message
            else:
                return "No return error!"
        except Exception, error:
            print error