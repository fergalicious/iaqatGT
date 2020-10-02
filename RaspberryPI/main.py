#!/usr/bin/python
import serial
import os
import datetime
import paramiko
from scp import SCPClient
#os.system('sudo mount -a')

#import argparse
comport = '/dev/ttyACM0' #Arduino serial may need to be changed
Time = datetime.datetime.now()
Date = Time.isoformat()[0:10]

filename = '/home/pi/Documents/Research/DataHistory/'+ Date + '.txt'
#print(filename)
if (not os.path.exists(filename)):
    with open(filename,'a') as f:
        f.write('Time,CO2,PM01,PM2.5,PM10,tempf,hum,H2S,C7H8,C2H5OH,NH3\n')
with serial.Serial( comport , 9600, timeout = 0.001) as arduino:
    while 1:
        if ( arduino.in_waiting > 116):
            Time = datetime.datetime.now()
            if (Time.isoformat()[0:10] != Date):
                Date = Time.isoformat()[0:10]
                filename = '/home/pi/Documents/Research/DataHistory/'+ Date + '.txt'
                with open(filename,'a') as f:
                    f.write('Time,CO2,PM01,PM2.5,PM10,tempf,hum,H2S,C7H8,C2H5OH,NH3\n')
            line = arduino.readline()
            filename = '/home/pi/Documents/Research/DataHistory/'+ Date + '.txt'
            with open(filename,'a+') as f:
                f.write(Time.strftime("%X") + ',')
                f.write(line.decode('utf-8'))
            with open('/home/pi/Documents/Research/current2.txt','w') as f1:
                f1.write(line.decode('utf-8'))
            #sudo apt-get install libffi-dev
            #pip install scp   
            ssh = paramiko.SSHClient()
            ssh.load_system_host_keys()
            #ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(hostname='64.227.11.204',username='joy')
            
               # SCPCLient takes a paramiko transport as its only argument
            scp= SCPClient(ssh.get_transport()) 
            scp.put('/home/pi/Documents/Research/current2.txt', '/var/www/html/current2.txt')
            #print '2'
            scp.close()
            #os.system('scp /home/pi/Documents/Research/current.txt joy@64.227.11.204:/var/www/html/current.txt')
            #f.write('\n')
            #f.close()
        
