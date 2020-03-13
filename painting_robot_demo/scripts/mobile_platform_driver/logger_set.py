#! /usr/bin/env python
# coding=utf-8

import os
import logging
from logging.handlers import RotatingFileHandler
import os
from math import pi
from colorama import Fore, Style
import time


class LoggerSetClass:
    def __init__(self,logfile_flag):
        self.logfile_flag=logfile_flag
        self.logger = logging.getLogger()## 创建一个logger
        self.logger_init()
    def logger_init(self):
        # 定义handler的输出格式
        # formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
        formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s")
        # Log等级总开关
        self.logger.setLevel(logging.DEBUG)
        if self.logfile_flag:
            # 创建log目录
            if not os.path.exists('./logfiles'):
                os.mkdir('./logfiles')

            # 创建一个handler，用于写入日志文件
            
            logfile = './logfiles/mobilerobot-'+time.strftime('%Y-%m-%d',time.localtime(time.time()))+'-logger-config-python.log'
            # 以append模式打开日志文件
            fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30)
            # 输出到file的log等级的开关
            fh.setLevel(logging.ERROR)
            # 为文件输出设定格式
            fh.setFormatter(formatter)
            # 设置文件输出到logger
            self.logger.addHandler(fh)
            # 再创建一个handler，用于输出到控制台
            ch = logging.StreamHandler()
            # 输出到console的log等级的开关
            ch.setLevel(logging.ERROR)
            # 控制台输出设定格式
            ch.setFormatter(formatter)
            # 设置控制台输出到logger
            self.logger.addHandler(ch)
        else:
            # 再创建一个handler，用于输出到控制台
            ch = logging.StreamHandler()
            # 输出到console的log等级的开关
            ch.setLevel(logging.INFO)
            # 控制台输出设定格式
            ch.setFormatter(formatter)
            # 设置控制台输出到logger
            self.logger.addHandler(ch)
    def loggererror(self,message):
        self.logger.error(Fore.RED + "[ERROR] - " + str(message) + Style.RESET_ALL)
    def loggerinfo(self,message,Color="GREEN"):
        if Color=='GREEN':
            self.logger.info(Fore.GREEN + "[INFO] - " + str(message) + Style.RESET_ALL)
        elif Color=='LIGHT_RED':
            self.logger.info(Fore.LIGHTRED_EX + "[INFO] - " + str(message) + Style.RESET_ALL)
    
    def loggerwarning(self,message):
        self.logger.warning()(Fore.YELLOW + "[WARNNING] - " + str(message) + Style.RESET_ALL)
def main():
    k=LoggerSetClass(1)
    k.loggerinfo("wocao-----",'LIGHT_RED')
    k.loggererror("ihihihih")
if __name__=="__main__":
    main()