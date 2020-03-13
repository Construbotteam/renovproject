#!/usr/bin/env python
# -*- coding: utf_8 -*-
import rospy
import sys
import binascii
import inspect
import serial
import time
from CRC_16_Check_CLB import *
from clb_command import *
from textwrap import wrap
from std_msgs.msg import String,UInt64MultiArray

class CLMBPKG:
    def __init__(self,nodename):
        self.readstringlength=7

        self.crc16=RobotCRC16()
        self.nodename=nodename
        self.plccmd=CLBDriverCommands() 
        self.Openmodbus_ok_flag=0
        self.buf = bytearray()
    def Init_node(self):
        rospy.init_node(self.nodename)

    def Send_message_to_port(self,ser,message):
        """
        :param message: String "010600000001480A" or "01 06 00 00 00 01 48 0A"
        :return: int list
        """
        # print("message",message)
        message_bytes = message.replace(" ",'').decode('hex')
        # print("message_bytes",message_bytes)
        # print str(hex(message_bytes))
        
        ser.write(message_bytes)
        ser.flushInput()
        ser.flushOutput()

        time.sleep(0.02)
        strt = ser.read(self.readstringlength).encode('hex')
        # endtime=time.time()
        # rospy.logerr("---Send_message_to_port--spend time -----$%s",endtime-starttime)
        readbuffer=[int(int(i, 16)) for i in wrap(strt, 2)]

        return readbuffer,message
    def Get_crc_16_str(self,cmd):
        return self.crc16.Combining_CRC_and_info(cmd)
    def Get_hex_list_from_OCT(self,data):
        if data<0:
            datahexstr=hex(int(data) & (2**16-1)).replace('0x','')
            datahexstr=datahexstr.zfill(len(datahexstr)+4-len(datahexstr))
            return [int(datahexstr[:2],16),int(datahexstr[2:],16)]
        elif data==0:
            return [0x00,0x00]
        else:
            # print data,type(data)
            datahexstr=hex(int(data)).replace('0x','')
            datahexstr=datahexstr.zfill(len(datahexstr)+4-len(datahexstr))
            return [int(datahexstr[:2],16),int(datahexstr[2:],16)]
    def Control_3DOF_Robot(self, ser, control_id, velocity, outputPulse):  # position control
            """

            :param master:
            :param control_id: 1-stand,2-rotation,3-climber
            :param velocity: 0-2500
            :param outputPulse: High 32位
            :return:
            """
            if control_id==1:
                #p001
                self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLD_DRIVER_MODEL_POSITION))#seting pos model
                #p290
 
                send290DataHEXlist=self.plccmd.HOLDING_INITIAL_P290_BASE_PULSE_DATA[:4]+self.Get_hex_list_from_OCT(outputPulse)
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(send290DataHEXlist)))#seting pos model

 
                send97DataHEXlist=self.plccmd.HOLDING_INITIAL_P97_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(velocity)
                self.Send_message_to_port(ser,self.Get_crc_16_str(send97DataHEXlist))#seting pos model

                send324DataHEXlist=self.plccmd.HOLDING_INITIAL_P324_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(1000)
                self.Send_message_to_port(ser,self.Get_crc_16_str(send324DataHEXlist))#seting pos model

                # self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLD_DRIVER_MODEL_POSITION))#seting pos model
                 
                # rospy.loginfo(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 0, 8))
                # # print type(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
                # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=6) #
                # #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
                # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 290,
                #                         output_value=outputPulse)  # High 16 10000 pulse 1 rpm,negtive up,positive up
                # #rospy.loginfo(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))  # Low 16bit
                # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity)  # internal velocity
                # # rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
                # # rospy.loginfo(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
                # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000)  # set fixed velocity
            if control_id==3:
                #p001
                self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.CLIMB_DRIVER_MODEL_POSITION))#seting pos model
                #p290
 
                send290DataHEXlist=self.plccmd.CLIMB_INITIAL_P290_BASE_PULSE_DATA[:4]+self.Get_hex_list_from_OCT(outputPulse)
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(send290DataHEXlist)))#seting pos model

 
                send97DataHEXlist=self.plccmd.CLIMB_INITIAL_P97_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(velocity)
                self.Send_message_to_port(ser,self.Get_crc_16_str(send97DataHEXlist))#seting pos model

                send324DataHEXlist=self.plccmd.CLIMB_INITIAL_P324_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(1000)
                self.Send_message_to_port(ser,self.Get_crc_16_str(send324DataHEXlist))#seting pos model
            if control_id==2:
                #p001
                self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.ROTATION_DRIVER_MODEL_POSITION))#seting pos model
                #p290
 
                send290DataHEXlist=self.plccmd.ROTATION_INITIAL_P290_BASE_PULSE_DATA[:4]+self.Get_hex_list_from_OCT(outputPulse)
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(send290DataHEXlist)))#seting pos model

 
                send97DataHEXlist=self.plccmd.ROTATION_INITIAL_P97_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(velocity)
                self.Send_message_to_port(ser,self.Get_crc_16_str(send97DataHEXlist))#seting pos model

                send324DataHEXlist=self.plccmd.ROTATION_INITIAL_P97_BASE_VELOCITY_DATA[:4]+self.Get_hex_list_from_OCT(1000)
                self.Send_message_to_port(ser,self.Get_crc_16_str(send324DataHEXlist))#seting pos model


    def Holding_Robot(self, ser, velocity, outputDistance, control_id=1):  # position control
        """

        :param master:
        :param velocity:
        :param outputPulse:Distance unit:m pos up,neg Down
        :param control_id:
        :return:
        """
        outputPulse = int(outputDistance*444.4)
        self.Control_3DOF_Robot(ser, control_id, velocity, -1.0*outputPulse)


    def Rotation_Robot(self, ser, velocity, outputDegree, control_id=2):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDegree: 0-3.14Degree,Positive disclockwise,Negtive clockwise
        :param control_id:
        :return:
        int(outputDegree)#
        """
        rospy.loginfo("0-3.14 Degree,Positive disclockwise,Negtive clockwise")
        
        outputPulse = int(outputDegree*8.73)#9.17
        self.Control_3DOF_Robot(ser, control_id, velocity, -1.0*outputPulse)


    def Climbing_Robot(self, ser, velocity, outputDistance, control_id=3):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDistance: 0-300cm
        :param control_id:
        :return:
        """

        rospy.loginfo("------climb robot,neg down,pos up")
        outputPulse = int(outputDistance *42.5)
        self.Control_3DOF_Robot(ser, control_id, velocity, -1.0*outputPulse)
    def Open_Stop_Enable_Driver(self, ser, control_id,stop_open_flag):
        if control_id==1:
            #p282
            if stop_open_flag==1:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLDING_DRIVER_P282_ENABALE)))#seting pos model
            if stop_open_flag==0:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.HOLDING_DRIVER_P282_DISENABALE)))#seting pos model      
        if control_id==3:
            #p282
            if stop_open_flag==1:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.CLIMB_DRIVER_P282_ENABALE)))#seting pos model
            if stop_open_flag==0:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.CLIMB_DRIVER_P282_DISENABALE)))#seting pos model      
        if control_id==2:
            #p282
            if stop_open_flag==1:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.ROTATION_DRIVER_P282_ENABALE)))#seting pos model
            if stop_open_flag==0:
                rospy.loginfo(self.Send_message_to_port(ser,self.Get_crc_16_str(self.plccmd.ROTATION_DRIVER_P282_DISENABALE)))#seting pos model      
                        
        # master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=stop_open_flag)  # enable

def main():
    clbpkg=CLMBPKG("tridof_pkg_node")
    open_serial_port_again_flag=0
    clbpkg.Init_node()

    climb_port = rospy.get_param("climb_port")
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port'), climb_port)

    # fetch the utterance parameter from our parent namespace
    climb_port_baudrate = rospy.get_param('climb_port_baudrate')
    rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_baudrate'), climb_port_baudrate)


    try:
        ser = serial.Serial(port=climb_port, baudrate=climb_port_baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
    except:
        rospy.logerr("Please check TRIDOF robot Usb port----")
        open_serial_port_again_flag=1

    count=0
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        rotation_homing_abs_encode_data=rospy.get_param("rotation_homing_abs_encode_data")
        # rospy.loginfo("%s is %s", rospy.resolve_name('rotation_homing_abs_encode_data'), rotation_homing_abs_encode_data)
        read_line_encode = rospy.get_param("read_line_encode")
        # rospy.loginfo("%s is %s", rospy.resolve_name('read_line_encode'), read_line_encode)
        rotation_abs_encode= rospy.get_param("rotation_abs_encode")
        # rospy.loginfo("%s is %s", rospy.resolve_name('rotation_abs_encode'), rotation_abs_encode)
        # rospy.logerr("rotation %s---rad--",1.1*(-rotation_homing_abs_encode_data+rotation_abs_encode)*2*pi/1024)
        
        climb_port_ok_flag = rospy.get_param("climb_port_ok_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('climb_port_ok_flag'), climb_port_ok_flag)

        close_all_3dof_climb_driver_flag = rospy.get_param("close_all_3dof_climb_driver_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('close_all_3dof_climb_driver_flag'), close_all_3dof_climb_driver_flag)

        enable_control_stand_bar = rospy.get_param("enable_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_control_stand_bar'), enable_control_stand_bar)

        enable_control_rotation = rospy.get_param("enable_control_rotation")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_control_rotation'), enable_control_rotation)

        enable_climb_control = rospy.get_param("enable_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_climb_control'), enable_climb_control)

        velocity_control_stand_bar = rospy.get_param("velocity_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('velocity_control_stand_bar'), velocity_control_stand_bar)

        velocity_control_rotation = rospy.get_param("velocity_control_rotation")
        # rospy.loginfo("%s is %s", rospy.resolve_name('velocity_control_rotation'), velocity_control_rotation)

        velocity_climb_control = rospy.get_param("velocity_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('velocity_climb_control'), velocity_climb_control)

        distance_control_stand_bar = rospy.get_param("distance_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('distance_control_stand_bar'), distance_control_stand_bar)

        rad_control_rotation = rospy.get_param("rad_control_rotation")
        # rospy.loginfo("%s is %s", rospy.resolve_name('rad_control_rotation'), rad_control_rotation)
        
        distance_climb_control = rospy.get_param("distance_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('distance_climb_control'), distance_climb_control)

        top_limit_switch_status = rospy.get_param("top_limit_switch_status")
        # rospy.loginfo("%s is %s", rospy.resolve_name('top_limit_switch_status'), top_limit_switch_status)

        mid_limit_switch_status = rospy.get_param("mid_limit_switch_status")
        # rospy.loginfo("%s is %s", rospy.resolve_name('mid_limit_switch_status'), mid_limit_switch_status)
        
        bottom_limit_switch_status = rospy.get_param("bottom_limit_switch_status")
        # rospy.loginfo("%s is %s", rospy.resolve_name('bottom_limit_switch_status'), bottom_limit_switch_status)

        enable_second_control_stand_bar = rospy.get_param("enable_second_control_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_control_stand_bar'), enable_second_control_stand_bar)


        enable_second_climb_control = rospy.get_param("enable_second_climb_control")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_climb_control'), enable_second_climb_control)

        enable_third_stand_bar = rospy.get_param("enable_third_stand_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_third_stand_bar'), enable_third_stand_bar)

        open_climb_flag = rospy.get_param("open_climb_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_control_stand_bar'), enable_second_control_stand_bar)

        open_hold_flag = rospy.get_param("open_hold_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_second_climb_control'), enable_second_climb_control)
        open_rotation_flag = rospy.get_param("open_rotation_flag")
        # rospy.loginfo("%s is %s", rospy.resolve_name('enable_third_stand_bar'), enable_third_stand_bar)

        if clbpkg.Openmodbus_ok_flag!=1 and climb_port_ok_flag==1:
            if top_limit_switch_status==1:
                if enable_second_control_stand_bar==0:
                    try:
                        clbpkg.Open_Stop_Enable_Driver(ser,1,0)
                    except:
                        rospy.logerr("some errors with top_limit_switch_status --- hold--0")
                    rospy.set_param('open_hold_flag',0)

            if mid_limit_switch_status==1:
                if enable_second_climb_control==0:
                    try:
                        clbpkg.Open_Stop_Enable_Driver(ser,3,0)
                    except:
                        rospy.logerr("some errors with mid_limit_switch_status--- climb--0")
                    rospy.set_param('open_hold_flag',0)

            if read_line_encode!=0 and read_line_encode<0.35:#0.39-->-80---0.44--->0
                if enable_second_control_stand_bar==0:
                    try:
                        clbpkg.Open_Stop_Enable_Driver(ser,1,0)
                    except:
                        rospy.logerr("some errors with read_line_encode--- hold--line encode")
                    rospy.set_param('open_hold_flag',0)
                else:
                    pass
                # pass
            if bottom_limit_switch_status==1:
                if enable_third_stand_bar==0:
                    try:
                        clbpkg.Open_Stop_Enable_Driver(ser,1,0)
                    except:
                        rospy.logerr("some errors with bottom_limit_switch_status--- hold")

                    rospy.set_param('open_hold_flag',0)

            if enable_control_stand_bar==1:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,1,1)
                except:
                    rospy.logerr("some errors with enable_control_stand_bar--- hold--1")

            if enable_control_stand_bar==2:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,1,0)
                except:
                    rospy.logerr("some errors with enable_control_stand_bar--- hold--0")
                rospy.set_param('open_hold_flag',0)

            if enable_control_rotation==1:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,2,1)
                except:
                    rospy.logerr("some errors with enable_control_rotation==1")
                # rospy.set_param('enable_control_rotation',0)
                # open_rotation_flag=1
                
            if enable_control_rotation==2:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,2,0)
                except:
                    rospy.logerr("some errors with enable_control_rotation--- rotation--0")                
                rospy.set_param('enable_control_rotation',0)
                # open_rotation_flag=0
                rospy.set_param('open_rotation_flag',0)
            if enable_climb_control==1:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,3,1)
                except:
                    rospy.logerr("some errors with enable_climb_control--- climb--1") 
                rospy.set_param('enable_climb_control',0)
                # open_climb_flag=1
            if enable_climb_control==2:
                try:
                    clbpkg.Open_Stop_Enable_Driver(ser,3,0)
                except:
                    rospy.logerr("some errors with enable_climb_control==2--- climb--0")                     
                rospy.set_param('enable_climb_control',0)
                # open_climb_flag=0
                rospy.set_param('open_climb_flag',0)
            #set velocity
            if open_hold_flag==1:
                try:
                    clbpkg.Holding_Robot(ser,velocity_control_stand_bar,distance_control_stand_bar)
                except:
                    rospy.logerr("something errro with open_hold_flag----")
                # open_hold_flag=0
                rospy.set_param('open_hold_flag',0)

            if open_climb_flag==1:
                # try:
                clbpkg.Climbing_Robot(ser,velocity_climb_control,distance_climb_control)
                # except:
                #     rospy.logerr("something errro with open_climb_flag----")
                # open_climb_flag=0
                rospy.set_param('open_climb_flag',0)
            if open_rotation_flag==1:
                try:
                    clbpkg.Rotation_Robot(ser,velocity_control_rotation,rad_control_rotation) 
                except:
                    rospy.logerr("something errro with open_rotation_flag----")
                rospy.set_param('open_rotation_flag',0)
                # open_rotation_flag=0       
        else:
            try:
                ser = serial.Serial(port=climb_port, baudrate=climb_port_baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
            except:
                rospy.loginfo("Please check TRIDOF ROBOT Usb port----,I will reconnect after three seconds-----")
                open_serial_port_again_flag=1
                time.sleep(3)
            
        rate.sleep()
if __name__ == "__main__":
    main()