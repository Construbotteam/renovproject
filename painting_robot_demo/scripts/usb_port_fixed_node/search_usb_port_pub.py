#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import String
import serial
from time import sleep
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from textwrap import wrap
import os,sys
import getpass

class SerarchUSB():
    def __init__(self,nodename):
        self.nodename=nodename
        self.iostatebuff=[]
        self.readstringlength=8
        self.pwd=0
    def Init_node(self):
        rospy.init_node(self.nodename)
    def password(self,):
        password = getpass.getpass("Please input the administrator's password: ")
        self.pwd=password
        return password
 
    def command(self,cmd):
        text = os.popen( "echo %s | sudo -S %s" % (self.pwd,cmd) ).read()
        return text
    def search_modbus_port(self,port,Baudrate):
        """
        id : 1---->stand bar
        id : 2----->rotation
        id : 3------>Upper and lower climbing pole

        """
        try:
            #Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=port, baudrate=Baudrate, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(1.0)
            master.set_verbose(True)
            buf1=master.execute(1, cst.READ_HOLDING_REGISTERS, 0, 8)
            buf2=master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8)
            buf3=master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 8)

            rospy.loginfo(buf1)
            rospy.loginfo(buf2)
            rospy.loginfo(buf3)
            if len(buf1)!=0:
                return True
        except:# modbus_tk.modbus.ModbusError as exc:
            
            #rospy.logerr("%s- Code=%d", exc, exc.get_exception_code())
            return False
    def Send_message_to_port(self,ser,message):
        """
        :param message: String "010600000001480A" or "01 06 00 00 00 01 48 0A"
        :return: int list
        """
        message_bytes = message.replace(" ",'').decode('hex')
        # print str(hex(message_bytes))
        ser.write(message_bytes)
        ser.flushInput()
        ser.flushOutput()
        time.sleep(0.01)
        strt = ser.read(self.readstringlength).encode('hex')
        readbuffer=[int(int(i, 16)) for i in wrap(strt, 2)]
        return readbuffer,strt
    def search_plc_port(self,port,Baudrate,Message):
        ser = serial.Serial(port, Baudrate, timeout=0.5)  #/dev/ttyUSB0
        readbuffer,strt=self.Send_message_to_port(ser,Message)
        print(readbuffer,strt)
        print('port',port)
        if len(readbuffer)!=0 and readbuffer[0]==4:
            if readbuffer[-1]!=None:
                ser.close()
                return True
            else:
                ser.close()
                return False
        ser.close()

def main():
    iob=SerarchUSB("search_port_node")
    iob.Init_node()
    iob.password()

    port_list=[]
    imu_port = rospy.get_param("imu_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('imu_port'), imu_port)
    port_list.append(imu_port)
    # fetch the utterance parameter from our parent namespace
    imu_port_baudrate = rospy.get_param('imu_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('imu_port_baudrate'), imu_port_baudrate)

    plc_port = rospy.get_param("plc_port")
    # print(type(plc_port))
    rospy.loginfo("%s is %s", rospy.resolve_name('plc_port'), plc_port)
    port_list.append(plc_port)
    # fetch the utterance parameter from our parent namespace
    plc_port_baudrate = rospy.get_param('plc_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)

    climb_port = rospy.get_param("climb_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)
    port_list.append(climb_port)
    for i in port_list:
        iob.command("chmod +777 "+i)
    # fetch the utterance parameter from our parent namespace
    climb_port_baudrate = rospy.get_param('climb_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)
    port_delete_plc_port=[]
    
    for i in port_list:
        if iob.search_plc_port(i,plc_port_baudrate,"040300050001945e"):
            rospy.loginfo("This is plc port")
            rospy.set_param("plc_port",i)
            rospy.set_param("plc_port_ok_flag",1)
        else:
            port_delete_plc_port.append(i)
            time.sleep(0.5)
    port_delete_plc_climb_port=[]
    for i in port_delete_plc_port:
        if iob.search_modbus_port(i,climb_port_baudrate):
            rospy.loginfo("This is climb port")
            rospy.set_param("climb_port",i)
            rospy.set_param("climb_port_ok_flag",1)
        else:
            port_delete_plc_climb_port.append(i)
    rospy.set_param("imu_port",port_delete_plc_climb_port[0])
    rospy.set_param("imu_port_ok_flag",1)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        imu_port = rospy.get_param("imu_port")
        climb_port = rospy.get_param("climb_port")
        plc_port_ok_flag=rospy.get_param("plc_port_ok_flag")
        imu_port_ok_flag=rospy.get_param("imu_port_ok_flag")
        climb_port_ok_flag=rospy.get_param("climb_port_ok_flag")
        plc_port = rospy.get_param("plc_port")
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_ok_flag'), plc_port_ok_flag)
        rospy.loginfo("%s is %s", rospy.resolve_name('imu_port_ok_flag'), imu_port_ok_flag)
        rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_ok_flag'), climb_port_ok_flag)
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)
        rospy.loginfo("%s is %s", rospy.resolve_name('imu_port'), imu_port)
        rospy.loginfo("%s is %s", rospy.resolve_name('imu_port_baudrate'), imu_port_baudrate)
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port'), plc_port)
        rospy.loginfo("%s is %s", rospy.resolve_name('plc_port_baudrate'), plc_port_baudrate)
        rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)
        rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)
        rate.sleep()
if __name__ == '__main__':
    main()