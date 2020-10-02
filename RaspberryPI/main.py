#!/usr/bin/python
import serial
import os
import datetime
import paramiko
from scp import SCPClient
#os.system('sudo mount -a')

#import argparse
comport = '/dev/ttyACM0' #location need to be changed
Time = datetime.datetime.now()
Date = Time.isoformat()[0:10]

filename = '/home/pi/Documents/Research/'+ Date + '.txt'
#print(filename)
if (not os.path.exists(filename)):
    with open(filename,'a') as f:
        f.write('Time,CO2,PM01,PM2.5,PM10,tempf,tempc,hum,TGS_1,C7H8,H2S,NH3,C2H5OH,HCHO,O3_1,O3\n')
with serial.Serial( comport , 9600, timeout = 0.001) as arduino:
    while 1:
        if ( arduino.in_waiting > 116):
            Time = datetime.datetime.now()
            if (Time.isoformat()[0:10] != Date):
                Date = Time.isoformat()[0:10]
                filename = '/home/pi/Documents/Research/'+ Date + '.txt'
                with open(filename,'a') as f:
                    f.write('Time,CO2,PM01,PM2.5,PM10,tempf,tempc,hum,TGS_1,C7H8,H2S,NH3,C2H5OH,HCHO,O3_1,O3\n')
            line = arduino.readline()
            filename = '/home/pi/Documents/Research/'+ Date + '.txt'
            with open(filename,'a+') as f:
                f.write(Time.strftime("%X") + ',')
                f.write(line.decode('utf-8'))
            with open('/home/pi/Documents/Research/current.txt','w') as f1:
                f1.write(line.decode('utf-8'))
               
            ssh = paramiko.SSHClient()
            ssh.load_system_host_keys()
            #ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(hostname='64.227.11.204',username='joy')
            
               # SCPCLient takes a paramiko transport as its only argument
            scp= SCPClient(ssh.get_transport()) 
            scp.put('/home/pi/Documents/Research/current.txt', '/var/www/html/current.txt')
            #print '2'
            scp.close()
            #os.system('scp /home/pi/Documents/Research/current.txt joy@64.227.11.204:/var/www/html/current.txt')
            #f.write('\n')
            #f.close()
        
