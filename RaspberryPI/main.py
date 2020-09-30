#!/usr/bin/python
import serial
import os
import datetime
os.system('sudo mount -a')

#import argparse
#determine which port connect to arduino
comport = '/dev/ttyACM0' #location need to be changed

#determine time and date
Time = datetime.datetime.now()
Date = Time.isoformat()[0:10]

#create filename
filename = '/home/pi/prismdrive/public_html/'+ Date + '.txt'
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
                filename = '/home/pi/prismdrive/public_html/'+ Date + '.txt'
                with open(filename,'a') as f:
                    f.write('Time,CO2,PM01,PM2.5,PM10,tempf,tempc,hum,TGS_1,C7H8,H2S,NH3,C2H5OH,HCHO,O3_1,O3\n')
            line = arduino.readline()
            filename = '/home/pi/prismdrive/public_html/'+ Date + '.txt'
            with open(filename,'a') as f:
#                print(Time.strftime("%X") + ',' + line.decode('utf-8'))
                f.write(Time.strftime("%X") + ',')
                f.write(line.decode('utf-8'))
            with open('/home/pi/prismdrive/public_html/current.txt','w') as f1:
                f1.write(line.decode('utf-8'))
            
            #f.write('\n')
            #f.close()
        
