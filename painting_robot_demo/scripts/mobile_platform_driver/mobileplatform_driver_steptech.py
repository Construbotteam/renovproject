#! /usr/bin/env python
# coding=utf-8
import sys
sys.path.append("..")
from ctypes import *
import yaml
import os
from math import *
from logger_set import *
from mobilecommand import *
from can_analysis_driver import *
import time
class MobilePlatformDriver():
    def __init__(self,):
        self.CanAnalysis=CanAnalysisDriver()
        # self.CanAnalysis.Init_Can_All()
        self.MobileDriver_Command=MobileDriverCommands()
        self.logger=LoggerSetClass(1)
        self.Datalen=8
        ###walking
        self.wheel_R=0.15/2
        self.Driver_walk_encode_fl=0
        self.Driver_walk_encode_fr=0
        self.Driver_walk_encode_rl=0
        self.Driver_walk_encode_rr=0   
        #walking velocity
        self.Driver_walk_velocity_encode_fl=0
        self.Driver_walk_velocity_encode_fr=0
        self.Driver_walk_velocity_encode_rl=0
        self.Driver_walk_velocity_encode_rr=0      
        ###steer
        self.Driver_steer_encode_fl=0
        self.Driver_steer_encode_fr=0
        self.Driver_steer_encode_rl=0
        self.Driver_steer_encode_rr=0
        self.Driver_steer_encode_fl_original=self.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['fl']
        self.Driver_steer_encode_fr_original=self.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['fr']
        self.Driver_steer_encode_rl_original=self.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['rl']
        self.Driver_steer_encode_rr_original=self.CanAnalysis.yamlDic['Homing_driver_fix_encoder_data']['rr']
        ####ROS
        self.Driver_walk_status_byte_encode_fl=0
        self.Driver_walk_status_byte_encode_fr=0
        self.Driver_walk_status_byte_encode_rl=0
        self.Driver_walk_status_byte_encode_rr=0      
        ###steer
        self.Driver_steer_status_byte_encode_fl=0
        self.Driver_steer_status_byte_encode_fr=0
        self.Driver_steer_status_byte_encode_rl=0
        self.Driver_steer_status_byte_encode_rr=0

    def Init_can(self):
        self.CanAnalysis.Init_Can_All() #初始化can分析仪，注意这里没有关闭分析仪
    def Open_driver_can_Node(self,IDD,Frames_Length):
        enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,0,self.Datalen,self.MobileDriver_Command.OPEN_CAN_NODE_DRIVER)
        # time.sleep(0.0015)
    def Clear_all_error_without_disable_driver(self):
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],0)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],0)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],0)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],0)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],1)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],1)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],1)
        Mc03=self.Enable_Clear_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],1)

    def Close_driver_can_Node(self,IDD,Frames_Length):
        enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,0,self.Datalen,self.MobileDriver_Command.CLOSE_CAN_NODE_DRIVER)
        # time.sleep(0.0015)
    def Enable_Motor_controller(self,IDD,Frames_Length,flag):
        """
        flag=1,velocity
        flag=0,postion
        """
        if flag:
            enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_DRIVER_SET_VELOCITY)
            # time.sleep(0.0015)
        else:
            enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.ENABLE_DRIVER_SET_POSITION)
            # time.sleep(0.0015)
    def Enable_Clear_Motor_controller(self,IDD,Frames_Length,flag):
        """
        flag=1,velocity
        flag=0,postion
        """
        if flag:
            enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.CLEAR_ERROR_WITHOUT_DISABLE)
            # time.sleep(0.0015)
        else:
            enbalecomd1=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.CLEAR_ERROR_WITHOUT_DISABLE)
            # time.sleep(0.0015)        
    def Enable_Steering_Controller(self,flag):
        self.logger.loggerinfo("Enable Steering Motor Controller","GREEN")
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)

    def Enable_Walking_Controller(self,flag):
        self.logger.loggerinfo("Enable Walking Motor Controller","GREEN")
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
        Mc03=self.Enable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'],flag)
        # time.sleep(0.001)
    def Enable_Motor_Controller_All(self):
        self.logger.loggerinfo("Enable Steering and Walking Motor Controller","GREEN")
        self.Enable_Steering_Controller(0)
        self.Enable_Walking_Controller(1)
    def Disable_Motor_controller(self,IDD,Frames_Length):
        
        disablecomd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],Frames_Length,IDD,self.Datalen,self.MobileDriver_Command.DISABLE_DRIVER)
        time.sleep(0.001)
        if disablecomd:
            strinfo="motor controller ID:"+str(IDD)+'is Disabled!!!'
            self.logger.loggerinfo(strinfo)
        return True
    def Disable_ALL_Motor_Controller(self):
        
        Mc01=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc01:
            self.logger.loggerinfo("Disable Front left Walking Motor Controller","GREEN")
        Mc02=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc02:
            self.logger.loggerinfo("Disable Front right Walking Motor Controller","GREEN")
        Mc03=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc03:
            self.logger.loggerinfo("Disable Rear Left Walking Motor Controller","GREEN")
        Mc04=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc04:
            self.logger.loggerinfo("Disable Rear Right Walking Motor Controller","GREEN")
        Mc05=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc05:
            self.logger.loggerinfo("Disable Front left Walking Motor Controller","GREEN")
        Mc06=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc06:
            self.logger.loggerinfo("Disable Front right Walking Motor Controller","GREEN")
        Mc07=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc07:
            self.logger.loggerinfo("Disable Rear Left Walking Motor Controller","GREEN")
        Mc08=self.Disable_Motor_controller(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo1'],self.CanAnalysis.yamlDic['Frames_Length'])
        time.sleep(0.001)
        if Mc08:
            self.logger.loggerinfo("Disable Rear Right Walking Motor Controller","GREEN")

    
    def Send_Control_Command(self,IDD,Commands):
        
        mod_cmd=self.CanAnalysis.Can_Transmit(self.CanAnalysis.yamlDic['nCanId'],self.CanAnalysis.yamlDic['Frames_Length'],IDD,self.Datalen,Commands)
        # time.sleep(0.0015)
        if mod_cmd:
            infostr='Command:'+str(Commands)+'send Ok!!!'
            self.logger.loggerinfo(infostr)
            return True
        else:
            self.logger.loggererror("Please Check your controller or USB CAN Analysis!!!!")
            return False
    def Send_trapezoid_Velocity(self,Velocity_data):
        self.Send_Velocity_Driver(Velocity_data,1,self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo2'])
        self.Send_Velocity_Driver(Velocity_data,1,self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo2'])
        self.Send_Velocity_Driver(Velocity_data,1,self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo2'])
        self.Send_Velocity_Driver(Velocity_data,1,self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo2'])
    def Send_YAML_COMMAND_To_Motor(self,flag,cmd):
        if flag:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
        elif flag==0:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
        else:
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
            self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'],cmd)
            time.sleep(0.001)
    def Send_Velocity_Driver(self,VelocityData,Steer_Walking,Driver_Id):
        """
        Unit:RPM/MIN
        """
        command=self.Caculate_Velocity_Command(int(VelocityData),Steer_Walking)
        self.Send_Control_Command(Driver_Id,command)
        # time.sleep(0.0015)
    def Send_Position_Driver(self,PosistionData,Driver_Id):
        command=self.Caculate_Position_Command(int(PosistionData))
        self.Send_Control_Command(Driver_Id,command)
        # time.sleep(0.0015)
    def Judge_data_NOT_POSITIVE_OR_Negative(self,octdata):
        status_byte=bin(octdata).replace("0b",'')
        status_byte=status_byte.zfill(len(status_byte)+8-len(status_byte))
        if status_byte[0]=='1':
            return True
        else:
            return False
    def Judge_Motor_Homing(self,status_binlist):
        if status_binlist[0]=='1':
            return True
        else:
            return False    
    def Get_Driver_Control_Status_16Byte(self,Octlist):
        """
        状态字节显示驱动器的状态
        bit0：准备上电
        bit1：已上电
        bit2：使能
        bit3：故障
        bit4：禁止输出电压
        bit5：快速停止
        bit6：上电禁止 bit7：警告
        bit8：内部保留
        bit9：远程控制
        bit10：目标位置到
        bit11：内部限位激活
        bit12：脉冲响应
        bit13：跟随误差/原点错误
        bit14：找到电机励磁
        bit15：原点找到
        """
        temp=[]
        for i in Octlist:
            temp.append(i)
        # return bin((temp[2]<<8|temp[1])&0xFFFF)
        status_byte=bin((temp[2]<<8|temp[1])&0xFFFF).replace('0b','')
            # print velocity
        status_byte=status_byte.zfill(len(status_byte)+16-len(status_byte))
        return status_byte

    def OCT_List_To_Oct_Four(self,Octlist,flag):
        """
        flag=0:velocity
        flag=1:position
        """
        # print "Hexstrlist",Hexstrlist
        temp=[]
        for i in Octlist:
            temp.append(i)
        # print "temp",temp
        if flag:
            if self.Judge_data_NOT_POSITIVE_OR_Negative(temp[7]):
                return -1*(~(temp[7]<<24|temp[6]<<16|temp[5]<<8|temp[4]-1)&0xFFFFFFFF)
            else:
                return ((temp[7]<<24|temp[6]<<16|temp[5]<<8|temp[4])&0xFFFFFFFF)
        elif flag==0:
            if self.Judge_data_NOT_POSITIVE_OR_Negative(temp[3]):
                return -1*(~(temp[3]<<24|temp[2]<<16|temp[1]<<8|temp[0]-1)&0xFFFFFFFF)
            else:
                return ((temp[3]<<24|temp[2]<<16|temp[1]<<8|temp[0])&0xFFFFFFFF)
        else:
            pass
    def Read_sensor_data_from_driver(self,):
        position_data_walk_vel_fl=self.Send_Control_Command(self.CanAnalysis.yamlDic['sync_data_ID'],self.MobileDriver_Command.ZERO_COMMAND)
        RecNum=self.CanAnalysis.Can_GetReceiveNum(0)
        # time.sleep(0.0015)
        if RecNum!=None:
            ret,kk=self.CanAnalysis.Can_New_Receive(0,RecNum)
            time.sleep(0.05)
            if ret:
                for i in range(RecNum):
                    if kk[i].ID==0x181:
                        # print "walking fl valid model and status  ID",hex(kk[i].ID)
                        # print "walking fl valid model and status  ID--------",self.List_to_HEXList(list(kk[i].Data))
                        # print kk[i].Data
                        self.Driver_walk_status_byte_encode_fl=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                        # print "walking fl",self.Get_Driver_Control_Status_16Byte(kk[i].Data)
                    if kk[i].ID==0x182:
                        # print "walking fr valid model and status ID",hex(kk[i].ID)
                        # print "walking fr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking fr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                        self.Driver_walk_status_byte_encode_fr=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                        # print "walking fr",self.Get_Driver_Control_Status_16Byte(kk[i].Data)
                    if kk[i].ID==0x183:
                        # print "walking rl valid model and status ID",hex(kk[i].ID)
                        # print "walking rl valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking rl hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                        self.Driver_walk_status_byte_encode_rl=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                        # print "walking rl",self.Get_Driver_Control_Status_16Byte(kk[i].Data)

                    if kk[i].ID==0x184:
                        # print "walking rr valid model and status ID",hex(kk[i].ID)
                        # print "walking rr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking rr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),0)
                        self.Driver_walk_status_byte_encode_rr=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))  
                        # print "walking rr",self.Get_Driver_Control_Status_16Byte(kk[i].Data)

                    if kk[i].ID==0x185:
                        # print "steering fl valid model and status ID",hex(kk[i].ID)
                        # print "steering fl valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering fl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_steer_status_byte_encode_fl=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                        # print "steering fl",self.Get_Driver_Control_Status_16Byte(kk[i].Data)

                    if kk[i].ID==0x186:
                        # print "steering fr valid model and status ID",hex(kk[i].ID)
                        # print "steering fr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering fr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_steer_status_byte_encode_fr=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                        # print "steering fr",self.Get_Driver_Control_Status_16Byte(kk[i].Data)

                    if kk[i].ID==0x187:
                        # print "steering rl valid model and status ID",hex(kk[i].ID)
                        # print "steering rl valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering rl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        ###steer

        
        
                        self.Driver_steer_status_byte_encode_rr=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                        # print "steering rl",self.Get_Driver_Control_Status_16Byte(kk[i].Data)

                    if kk[i].ID==0x188:
                        # print "steering rr valid model and status ID",hex(kk[i].ID)
                        # print "steering rr valid model and status--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering rr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        # print "steering rr",self.Get_Driver_Control_Status_16Byte(kk[i].Data)
                        self.Driver_steer_status_byte_encode_rr=list(self.Get_Driver_Control_Status_16Byte(kk[i].Data))
                    if kk[i].ID==0x281:
                        # print "walking fl data ID",hex(kk[i].ID)
                        # print "walking fl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking fl hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_walk_velocity_encode_fl=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x282:
                        # print "walking fr data ID",hex(kk[i].ID)
                        # print "walking fr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking fr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_walk_velocity_encode_fr=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x283:
                        # print "walking rl data ID",hex(kk[i].ID)
                        # print "walking rl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking rl hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_walk_velocity_encode_rl=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x284:
                        # print "walking rr data ID",hex(kk[i].ID)
                        # print "walking rr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "walking rr hexdata velocity --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_walk_velocity_encode_rr=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x285:
                        # print "steering fl data ID",hex(kk[i].ID)
                        # print "steering fl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering fl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_steer_encode_fl=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x286:
                        # print "steering fr data ID",hex(kk[i].ID)
                        # print "steering fr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering fr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_steer_encode_fr=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x287:
                        # print "steering rl data ID",hex(kk[i].ID)
                        # print "steering rl hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering rl hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_steer_encode_rl=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                    if kk[i].ID==0x288:
                        # print "steering rr data ID",hex(kk[i].ID)
                        # print "steering rr hexdata--------",self.List_to_HEXList(list(kk[i].Data))
                        # print "steering rr hexdata position --------",self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
                        self.Driver_steer_encode_rr=self.OCT_List_To_Oct_Four(list(kk[i].Data),1)
    def List_to_HEXList(self,Listdata):
        temp=[]
        for i in Listdata:
            temp.append(hex(i))
        return temp
    def HEX_String_List_To_Oct(self,Hexstrlist):
        temp=[]
        for i in Hexstrlist:
            temp.append("0x{:02x}".format(i))
        # print temp
        hex03=temp[3]
        hex04=temp[4]
        newtemp=int(''.join([hex04,hex03]).replace('0x',''),16)#str([hex04,hex03])
        return newtemp
    def Caculate_Velocity_Command(self,VelocityData,flag):

        BASIC=self.MobileDriver_Command.BASIC_TARGET_COMMAND#(0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00)

        NEWCMD=()
        if flag:#steer
            VelocityData=VelocityData*512*4096/1875
        else:
            VelocityData=VelocityData*512*10000/1875
        if VelocityData<0:
            velocity=hex(VelocityData & 0xFFFFFFFF)
            newvelocity=[velocity[i:i+2] for i in range(0,len(velocity), 2)]
            # print newvelocity
            NEWCMD=(int(newvelocity[4],16),int(newvelocity[3],16),int(newvelocity[2],16),int(newvelocity[1],16))+BASIC[4:]
            # print(newvelocity,NEWCMD)
            return NEWCMD

        elif VelocityData>0:
            velocity=hex(VelocityData).replace('0x','')
            # print velocity
            newvelocity=velocity.zfill(len(velocity)+8-len(velocity))
            # print newvelocity
            tempnewvelocity=[newvelocity[i:i+2] for i in range(0,len(newvelocity), 2)]
            # print tempnewvelocity
            NEWCMD=(int(tempnewvelocity[3],16),int(tempnewvelocity[2],16),int(tempnewvelocity[1],16),int(tempnewvelocity[0],16))+BASIC[5:]
            # print NEWCMD
            return NEWCMD

        else:
            return NEWCMD
    def caculate_velocity(self,vel):
        # return (60*vel)/(0.15*pi)
        return (60*vel*20.15)/(2*pi*self.wheel_R)
    def rad_to_pulse(self,rad):
        #rad 弧度
        return (rad*(220*1024*4.0)/(2.0*pi))
    def Send_position_to_four_steering_wheel(self,postion_list):
        """
        rad_target--->rad
        """
        OutputPulse=postion_list
        print "PositionData:",OutputPulse
        self.Send_Position_Driver(int(OutputPulse[0]),self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[1]),self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[2]),self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[3]),self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
    
    def Send_same_degree_position_to_four_steering_wheel(self,ratation_flag,rad_target):
        """
        rad_target--->rad
        """
        OutputPulse=self.output_pulse_position_control(ratation_flag,rad_target)#([-1.0,-1.0,-1.0,-1.0],pi/2)
        print "PositionData:",OutputPulse
        self.Send_Position_Driver(int(OutputPulse[0]),self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[1]),self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[2]),self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[3]),self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
    def Send_diff_degree_position_to_four_steering_wheel(self,ratation_flag,rad_target):
        """
        rad_target--->rad list[rad_fl,rad_fr,rad_rl,rad_rr]
        """
        rad_fl=rad_target[0]
        rad_fr=rad_target[1]
        rad_rl=rad_target[2]
        rad_rr=rad_target[3]
        OutputPulse=self.output_pulse_position_control_multi_rad(ratation_flag,rad_fl,rad_fr,rad_rl,rad_rr)#([-1.0,-1.0,-1.0,-1.0],pi/2)
        print "PositionData:",OutputPulse
        self.Send_Position_Driver(int(OutputPulse[0]),self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[1]),self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[2]),self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Position_Driver(int(OutputPulse[3]),self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo3'])
        
    def Send_same_velocity_to_four_walking_wheel(self,wheel_diretion_flg,speed_flag,speed):
        VelocityData= self.caculate_velocity(speed_flag*speed)#all motor 45degree velocity
        print "VelocityData:RPM/Min",VelocityData
        self.Send_Velocity_Driver(wheel_diretion_flg[0]*int(VelocityData),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Velocity_Driver(wheel_diretion_flg[1]*int(VelocityData),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
        self.Send_Velocity_Driver(wheel_diretion_flg[2]*int(VelocityData),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Velocity_Driver(wheel_diretion_flg[3]*int(VelocityData),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
    def Send_diff_velocity_to_four_walking_wheel(self,wheel_diretion_flg,speed_flag,speedfl,speedfr,speedrl,speedrr):
        """
        wheel_diretion_flg--->list
        speed_flag--->list
        """
        VelocityDatafl= self.caculate_velocity(speed_flag[0]*speedfl)#all motor 45degree velocity
        print "Send velocity command VelocityDatafl:RPM/Min",VelocityDatafl
        VelocityDatafr= self.caculate_velocity(speed_flag[0]*speedfr)#all motor 45degree velocity
        print "VelocityDatafr:RPM/Min",VelocityDatafr
        VelocityDatarl= self.caculate_velocity(speed_flag[0]*speedrl)#all motor 45degree velocity
        print "VelocityDatarl:RPM/Min",VelocityDatarl
        VelocityDatarr= self.caculate_velocity(speed_flag[0]*speedrr)#all motor 45degree velocity
        print "VelocityDatarr:RPM/Min",VelocityDatarr
        VelocityDatafl= self.caculate_velocity(speed_flag[0]*speedfl)#all motor 45degree velocity
        self.Send_Velocity_Driver(wheel_diretion_flg[0]*int(VelocityDatafl),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Velocity_Driver(wheel_diretion_flg[1]*int(VelocityDatafr),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['front_walking_right_rpdo']['rpdo']['rpdo3'])
        self.Send_Velocity_Driver(wheel_diretion_flg[2]*int(VelocityDatarl),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_left_rpdo']['rpdo']['rpdo3'])
        self.Send_Velocity_Driver(wheel_diretion_flg[3]*int(VelocityDatarr),0,self.CanAnalysis.yamlDic['walking_channel_pdo']['rear_walking_right_rpdo']['rpdo']['rpdo3'])
        
    def output_pulse_position_control(self,flag_list,degree_tar):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original
        return [oldpusle_fl+flag_list[0]*self.rad_to_pulse(degree_tar),oldpusle_fr+flag_list[1]*self.rad_to_pulse(degree_tar),oldpusle_rl+flag_list[2]*self.rad_to_pulse(degree_tar),oldpusle_rr+flag_list[3]*self.rad_to_pulse(degree_tar)]
    def degree_to_rad(self,degree_data):
        return (degree_data*pi)/180
    def Dec_to_RPM(self,DEC_data):
        return (DEC_data*1875.0)/(512*10000)
    def Pos_to_rad(self,pose_dec):
        #rad*(220*1024*4.0)/(2.0*pi)
        return pose_dec*(2.0*pi)/(220*1024*4.0)
    def output_pulse_position_control_multi(self,flag_list,rad_fl,rad_fr,rad_rl,rad_rr):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original
        return [oldpusle_fl+flag_list[0]*self.rad_to_pulse(self.degree_to_rad(rad_fl)),oldpusle_fr+flag_list[1]*self.rad_to_pulse(self.degree_to_rad(rad_fr)),oldpusle_rl+flag_list[2]*self.rad_to_pulse(self.degree_to_rad(rad_rl)),oldpusle_rr+flag_list[3]*self.rad_to_pulse(self.degree_to_rad(rad_rr))]
    def output_pulse_position_control_multi_rad(self,flag_list,rad_fl,rad_fr,rad_rl,rad_rr):
        """
        flag_list=[]
        """
        oldpusle_fl=self.Driver_steer_encode_fl_original
        oldpusle_fr=self.Driver_steer_encode_fr_original
        oldpusle_rl=self.Driver_steer_encode_rl_original
        oldpusle_rr=self.Driver_steer_encode_rr_original
        return [oldpusle_fl+flag_list[0]*self.rad_to_pulse(rad_fl),oldpusle_fr+flag_list[1]*self.rad_to_pulse(rad_fr),oldpusle_rl+flag_list[2]*self.rad_to_pulse(rad_rl),oldpusle_rr+flag_list[3]*self.rad_to_pulse(rad_rr)]
    
    def Command_Set_Homing_Model(self):
        Mc03=self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_left_rpdo']['rpdo']['rpdo1'],self.MobileDriver_Command.ENABLE_DRIVER_SET_HOMING)
        # time.sleep(0.001)
        Mc03=self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['front_steering_right_rpdo']['rpdo']['rpdo1'],self.MobileDriver_Command.ENABLE_DRIVER_SET_HOMING)
        # time.sleep(0.001)
        Mc03=self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_left_rpdo']['rpdo']['rpdo1'],self.MobileDriver_Command.ENABLE_DRIVER_SET_HOMING)
        # time.sleep(0.001)
        Mc03=self.Send_Control_Command(self.CanAnalysis.yamlDic['steering_channel_pdo']['rear_steering_right_rpdo']['rpdo']['rpdo1'],self.MobileDriver_Command.ENABLE_DRIVER_SET_HOMING)
        # time.sleep(0.001)
    def Caculate_Position_Command(self,PositionData):
        BASIC=self.MobileDriver_Command.BASIC_TARGET_COMMAND

        NEWCMD=()

        if PositionData<0:

                position=hex(PositionData & 0xFFFFFFFF)
                newposition=[position[i:i+2] for i in range(0,len(position), 2)]
                NEWCMD=BASIC[:4]+(int(newposition[4],16),int(newposition[3],16),int(newposition[2],16),int(newposition[1],16))
                # print newposition,NEWCMD
                return NEWCMD

        elif PositionData>0:
            position=hex(PositionData).replace('0x','')
            # print position
            if len(position)>=1:
                newposition=position.zfill(len(position)+8-len(position))
                # print newposition
                tempnewposition=[newposition[i:i+2] for i in range(0,len(newposition), 2)]
                # print tempnewposition
            else:
                pass

            NEWCMD=BASIC[:4]+(int(tempnewposition[3],16),int(tempnewposition[2],16),int(tempnewposition[1],16),int(tempnewposition[0],16))
            # print NEWCMD
            return NEWCMD

        else:
            return NEWCMD
def main():
    mpd=MobilePlatformDriver()
    count=1
    flag=0
    mpd.Init_can()
    mpd.Open_driver_can_Node(0x00000000,1)
    while 1:
        mpd.Read_sensor_data_from_driver()
        time.sleep(0.3)
    # time.sleep(1)
    # mpd.Enable_Motor_Controller_All()
    # # print mpd.Driver_steer_encode_fl,mpd.Driver_steer_encode_fr,mpd.Driver_steer_encode_rl,mpd.Driver_steer_encode_rr
    # time.sleep(1)
    # mpd.Read_sensor_data_from_driver()
    # # time.sleep(3)
    # # print mpd.Driver_steer_encode_fl,mpd.Driver_steer_encode_fr,mpd.Driver_steer_encode_rl,mpd.Driver_steer_encode_rr
    # mpd.Read_sensor_data_from_driver()
    # print "homing is ok----!!!"
    # while count:
    #     # mpd.Save_Parameter('1')
    #     # time.sleep(1)
    #     if flag==1:
    #         mpd.Send_YAML_COMMAND_To_Motor(3,mpd.MobileDriver_Command.ZERO_VELOCITY)
    #     else:
    #         mpd.Send_YAML_COMMAND_To_Motor(0,mpd.MobileDriver_Command.TEST_VELOCITY_STEERING)
    #         mpd.Send_YAML_COMMAND_To_Motor(1,mpd.MobileDriver_Command.TEST_VELOCITY_WALKING)
    #     count-=1
    #     mpd.Read_sensor_data_from_driver()
    # time.sleep(3)
    # mpd.Send_YAML_COMMAND_To_Motor(1,mpd.MobileDriver_Command.ZERO_VELOCITY)
    # mpd.Send_YAML_COMMAND_To_Motor(0,mpd.MobileDriver_Command.ZERO_VELOCITY)
    mpd.CanAnalysis.Can_VCICloseDevice()
if __name__=="__main__":
    main()