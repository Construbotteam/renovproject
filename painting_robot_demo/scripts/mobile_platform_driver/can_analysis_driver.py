#! /usr/bin/env python
# coding=utf-8
# import rospy
import sys
# sys.path.append("..")
from ctypes import *
import yaml
import os
from math import pi
from logger_set import *
from mobilecommand import *
import time
class VCI_BOARD_INFO(Structure):
    _fields_ = [('hw_Version',c_ushort),
                ('fw_Version',c_ushort),
                ('dr_Version',c_ushort),
                ('in_Version',c_ushort),
                ('irq_Num',c_ushort),
                ('can_Num',c_byte),
                ('str_Serial_Num',c_char*20),
                ('str_hw_Type',c_char*40),
                ('Reserved',c_ushort*4)
                                        ]
class VCI_CAN_OBJ(Structure):
    _fields_ = [('ID',c_uint),
                ('TimeStamp',c_uint),
                ('TimeFlag',c_byte),
                ('SendType',c_byte),
                ('RemoteFlag',c_byte),
                ('ExternFlag',c_byte),
                ('DataLen',c_byte),
                ('Data',c_ubyte*8),
                ('Reserved',c_ubyte*3)
                                        ]
class VCI_INIT_CONFIG(Structure):
    _fields_ = [('AccCode',c_uint),
                ('AccMask',c_uint),
                ('Reserved',c_uint),
                ('Filter',c_ubyte),
                ('Timing0',c_ubyte),
                ('Timing1',c_ubyte),
                ('Mode',c_ubyte)
                                        ]

class CanAnalysisDriver:
    def __init__(self):
        self.configname="mobileparameter.yaml"
        self.canlibpath="/data/ros/renov_robot_ws/src/painting_robot_demo/lib/libcontrolcan.so"#rospy.get_param('cansolib_path')
        self.OpreateCanAnalysis = cdll.LoadLibrary(self.canlibpath)#(str('/data/ros/yue_wk_2019/src/painting_robot_demo/lib/libcontrolcan.so'))
        self.canyamlconfig_path="/data/ros/renov_robot_ws/src/painting_robot_demo/config/"#rospy.get_param('canyaml_config_path')
        self.yamlDic={}
        self.Opreating_Yaml()# init yaml
        
        self.logger = LoggerSetClass(0)
    def Opreating_Yaml(self):

        yaml_path = self.canyamlconfig_path+self.configname#str("/data/ros/yue_wk_2019/src/mobile_robot/src/config/"+self.configname)
        # print yaml_path
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        file_data.close()
    def Can_VCIOpenDevice(self):
        Canstatus=self.OpreateCanAnalysis.VCI_OpenDevice(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],0)
        if Canstatus==0:
            self.logger.loggererror("Open Can Analysis device error")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Open Success!")
            self.logger.loggerinfo(Canstatus)
            return True
    def Can_VCICloseDevice(self):
        Canstatus=self.OpreateCanAnalysis.VCI_CloseDevice(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'])
        if Canstatus==0:
            self.logger.loggererror("Close can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Close Success!!")
            self.logger.loggerinfo(Canstatus)
            return True  
    def Can_VCIInitCan_PyInit(self,CanInd,VCI_INIT_CONFIG_STRUC):
        
        CanFuncStruc=self.OpreateCanAnalysis.VCI_InitCAN#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,c_uint,POINTER(VCI_INIT_CONFIG)]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,byref(VCI_INIT_CONFIG_STRUC))
    def Can_VCIInitCan(self,CanInd):
        config=VCI_INIT_CONFIG()
        config.AccCode = 0x00000000
        config.AccMask = 0xFFFFFFFF
        config.Filter = 0
        config.Mode = 0
        config.Timing0 = 0x00
        config.Timing1 = 0x1c
        #print(config)
        Canstatus=self.Can_VCIInitCan_PyInit(CanInd,config)
        if Canstatus==0:
            self.logger.loggererror("Init Can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Init Success!!")
            self.logger.loggerinfo(Canstatus)
            return True   
    def Can_ReadBoardInfo_PyInit(self,VCI_BOARD_INFO_STRUC):
        CanFuncStruc=self.OpreateCanAnalysis.VCI_ReadBoardInfo#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,POINTER(VCI_BOARD_INFO)]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],byref(VCI_BOARD_INFO_STRUC))
    def Can_ReadBoardInfo(self):
        config=VCI_BOARD_INFO()
        Canstatus=self.Can_ReadBoardInfo_PyInit(config)

        # print(config.can_Num)
        # print Canstatus
        if Canstatus==0:
            self.logger.loggererror("Read Board Info Can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Read Board Info Success!!")
            self.logger.loggerinfo(Canstatus)
            self.logger.loggerinfo("Can Analysis Board Info like:")
            self.logger.loggerinfo("hw_Version: "+str(config.hw_Version))
            self.logger.loggerinfo("fw_Version: "+str(config.fw_Version))
            self.logger.loggerinfo("dr_Version: "+str(config.dr_Version))
            self.logger.loggerinfo("in_Version: "+str(config.in_Version))
            self.logger.loggerinfo("irq_Num: "+str(config.irq_Num))
            self.logger.loggerinfo("can_Num: "+str(config.can_Num))
            self.logger.loggerinfo("str_Serial_Num: "+str(config.str_Serial_Num))
            self.logger.loggerinfo("str_hw_Type: "+str(config.str_hw_Type))
            self.logger.loggerinfo("Reserved: "+str(config.Reserved))

            return Canstatus 
    def Can_GetReceiveNum(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_GetReceiveNum(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd)
        if Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
            return Canstatus
        elif Canstatus==0:
            self.logger.loggererror("Can Analysis Get Receive Num No data")
            self.logger.loggererror(Canstatus)
            return Canstatus
        else:
            self.logger.loggerinfo("Can Analysis Get Receive Num Success!!")
            self.logger.loggerinfo(Canstatus)
            return Canstatus  
    def Can_ClearBuffer(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_ClearBuffer(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        if Canstatus==0:
            self.logger.loggererror("Clear Buffer Can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Clear Buffer Success!!")
            self.logger.loggerinfo(Canstatus)
            return True  
    def Can_StartCAN(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_StartCAN(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        if Canstatus==0:
            self.logger.loggererror("Start Can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Start Success!!")
            self.logger.loggerinfo(Canstatus)
            return True  
    def Can_ResetCAN(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_ResetCAN(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        if Canstatus==0:
            self.logger.loggererror("Reset Can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Reset Success!!")
            self.logger.loggerinfo(Canstatus)
            return True  
    def Can_Transmit_new_multi(self,CanInd,SenData_dic):
        try:
            Len=len(SenData_dic)
            vci_can_obj = (Len*VCI_CAN_OBJ)()
            for i in range(Len):
                vci_can_obj[i].ID=SenData_dic[IDD]
                vci_can_obj[i].RemoteFlag=0
                vci_can_obj[i].ExternFlag =0
                vci_can_obj[i].SendType = 0
                vci_can_obj[i].DataLen = 8
                vci_can_obj[i].Data=SenData_dic[SenData]
            Canstatus=self.OpreateCanAnalysis.VCI_Transmit(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,vci_can_obj,48)
            if Canstatus== -1:
                self.logger.loggererror("Can analysis offline!")
                self.logger.loggererror(Canstatus)
            elif Canstatus==0:
                self.logger.loggererror("Can Analysis Receive No data!!")
                self.logger.loggererror(Canstatus)
            else:
                self.logger.loggerinfo("Can Analysis Receive Success!!")
                self.logger.loggerinfo(Canstatus)
            return Canstatus,vci_can_obj
        except:
            pass
    def Can_Transmit_PyInit(self,CanInd,Length,VCI_CAN_OBJ_STRUC):
        CanFuncStruc=self.OpreateCanAnalysis.VCI_Transmit#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,c_uint,POINTER(VCI_CAN_OBJ),c_uint]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,byref(VCI_CAN_OBJ_STRUC),Length)
    def Can_Transmit(self,CanInd,Length,IDD,Datalen,SenData,):
        #for i in range(Length):
        config=VCI_CAN_OBJ()
        config.ID=IDD
        config.RemoteFlag=0
        config.ExternFlag =0
        config.SendType = 0
        config.DataLen = Datalen
        config.Data=SenData
        # ubyte_array = c_ubyte*8
        # a = ubyte_array(0x04,0x01,0x01,0x00)

        # ubyte_3array = c_ubyte*3
        # b = ubyte_3array(0, 0 , 0)
        # vci_can_obj_2 = VCI_CAN_OBJ(1, 0, 0, 0, 0, 0,  8, a, b)
        # print(SenData)
        # for i in range(0,Datalen):
        #     # print(type(SenData[i]))
        #     config.Data[i]=SenData[i]
            
        self.logger.loggerinfo(SenData)
        #len+=1
        Canstatus=self.Can_Transmit_PyInit(0,1,config)
        # Canstatus=self.Can_Transmit_PyInit(0,48,config)
        # time.sleep(0.1)
        if Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggererror(Canstatus)
        elif Canstatus==0:
            self.logger.loggererror("Can analysis Transmit No data")
            self.logger.loggererror(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Data Transmit Success!!")
            self.logger.loggerinfo(Canstatus)
            return Canstatus 
    def Can_New_Receive(self,CanInd,Len): 
        # ubyte_array=c_ubyte*8
        # a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
        # ubyte_3array = c_ubyte*3
        # b = ubyte_3array(0, 0 , 0)
        try:
            vci_can_obj = (Len*VCI_CAN_OBJ)()
            Canstatus=self.OpreateCanAnalysis.VCI_Receive(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,vci_can_obj,Len,0)
            if Canstatus== -1:
                self.logger.loggererror("Can analysis offline!")
                self.logger.loggererror(Canstatus)
            elif Canstatus==0:
                self.logger.loggererror("Can Analysis Receive No data!!")
                self.logger.loggererror(Canstatus)
            else:
                self.logger.loggerinfo("Can Analysis Receive Success!!")
                self.logger.loggerinfo(Canstatus)
            return Canstatus,vci_can_obj
        except:
            print("can device error--------maybe can device disconnected----please check")
    def Can_Receive_PyInit(self,CanInd,Len,WaitTime,VCI_CAN_OBJ_STRUC):
        CanFuncStruc=self.OpreateCanAnalysis.VCI_Receive#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,c_uint,POINTER(VCI_CAN_OBJ),c_ulong,c_int]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,byref(VCI_CAN_OBJ_STRUC),Len,WaitTime)

    def Can_Mult_Receive(self,CanInd,Len):
        config=(Len*VCI_CAN_OBJ)()

        # ubyte_array = c_ubyte*8
        # a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
        # ubyte_3array = c_ubyte*3
        # b = ubyte_3array(0, 0 , 0)
        # config = VCI_CAN_OBJ(0x0, 0, 0, 0, 0, 0,  8, a, b)

        Canstatus=self.Can_Receive_PyInit(CanInd,Len,0,config)
        if Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggererror(Canstatus)
        elif Canstatus==0:
            self.logger.loggererror("Can Analysis Receive No data!!")
            self.logger.loggererror(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Receive Success!!")
            self.logger.loggerinfo(Canstatus)
            return Canstatus,config
    def Can_Receive(self,CanInd,Len):
        config=VCI_CAN_OBJ()

        # ubyte_array = c_ubyte*8
        # a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
        # ubyte_3array = c_ubyte*3
        # b = ubyte_3array(0, 0 , 0)
        # config = VCI_CAN_OBJ(0x0, 0, 0, 0, 0, 0,  8, a, b)

        Canstatus=self.Can_Receive_PyInit(CanInd,Len,0,config)
        if Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggererror(Canstatus)
        elif Canstatus==0:
            self.logger.loggererror("Can Analysis Receive No data!!")
            self.logger.loggererror(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Receive Success!!")
            self.logger.loggerinfo(Canstatus)
            return Canstatus,config
    def Can_VCI_UsbDeviceReset(self):
        Canstatus=self.OpreateCanAnalysis.VCI_UsbDeviceReset(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],0)
        if Canstatus==0:
            self.logger.loggererror("Reset USB Can analysis device error!")
            self.logger.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.logger.loggererror("Can analysis offline!")
            self.logger.loggerinfo(Canstatus)
        else:
            self.logger.loggerinfo("Can Analysis Reset USB Success!!")
            self.logger.loggerinfo(Canstatus)
            return True 
    def Init_Can_All(self,):
        # try:
        #     if self.Can_VCIOpenDevice():
        #         pass
        #     else:
        #         pass
        # except:
        #     rospy.loggerinfo("------can analysis is initial ok---- or -------can analysis is already open-----")
        # try:
        #     self.Can_VCICloseDevice()
        # except:
        #     pass
        self.Can_VCIOpenDevice()
        self.Can_VCIInitCan(self.yamlDic['nCanId'])
        self.Can_StartCAN(self.yamlDic['nCanId'])
        self.Can_ResetCAN(self.yamlDic['nCanId'])
        self.Can_ReadBoardInfo()
        # else:
        #     pass
        #     self.logger.loggererror("------can analysis is initial ok---- or -------can analysis is already open-----")
def main():
    md=CanAnalysisDriver()
    # md.Opreating_Yaml()
    md.logger.loggerinfo(md.yamlDic,'GREEN')
    
    md.Can_VCIOpenDevice()
    md.Can_VCIInitCan(0)
    md.Can_StartCAN(0)
    md.Can_ResetCAN(0)
    md.Can_ReadBoardInfo()
    comd=MobileDriverCommands()

    md.Can_Transmit(0,1,1,8,comd.REQUEST_ENCODER_1)

    time.sleep(0.3)
    renum=md.Can_GetReceiveNum(0)
    ret,kk=md.Can_Receive(0,1)
    count=5
    if renum>0:
        while count:
            renum=md.Can_GetReceiveNum(0)
            print("Can_GetReceiveNum",renum)
            transmit_status=md.Can_Transmit(0,1,1,8,comd.REQUEST_ENCODER_1)
            if transmit_status:
                # time.sleep(0.1)
                ret,kk=md.Can_New_Receive(0,renum)
                if ret:
                    for i in range(len(kk)):
                        print('my data 1',list(kk[i].Data))

                    # if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    #     print('my data 1',list(kk.Data))
                    #     print(kk.DataLen)
            else:
                print('my data 2',list(kk.Data))
                print(kk.DataLen)
            # transmit_status=md.Can_Transmit(0,1,2,8,comd.REQUEST_ENCODER_2)
            # if transmit_status:
            #     # time.sleep(0.1)
            #     ret,kk=md.Can_Receive(0,1)
            #     if ret:
            #         if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
            #             print('my data 2',list(kk.Data))
            #             print(kk.DataLen)
            # else:
            #     print('my data 2',list(kk.Data))
            #     print(kk.DataLen)
            # transmit_status=md.Can_Transmit(0,1,3,8,comd.REQUEST_ENCODER_3)
            # if transmit_status:
            #     # time.sleep(0.1)
            #     ret,kk=md.Can_Receive(0,1)
            #     if ret:
            #         if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
            #             print('my data 3',list(kk.Data))
            #             print(kk.DataLen)
            # else:
            #     print('my data 3',list(kk.Data))
            #     print(kk.DataLen)
            # transmit_status=md.Can_Transmit(0,1,4,8,comd.REQUEST_ENCODER_4)
            # if transmit_status:
            #     # time.sleep(0.1)
            #     ret,kk=md.Can_Receive(0,1)
            #     if ret:
            #         if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
            #             print('my data 4',list(kk.Data))
            #             print(kk.DataLen)
            # else:
            #     print('my data 4',list(kk.Data))
            #     print(kk.DataLen)
            count-=1
    md.Can_VCICloseDevice()
    # md.OpreateCanAnalysis.VCI_CloseDevice(4, 0,0)
if __name__=="__main__":
    main()