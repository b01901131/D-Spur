from FK import *
import serial
import math
import time

PI = math.pi
RADIUS = 80

def usage():
    print "Usage : input 9 pose parameters."


ser = serial.Serial(
    port='/dev/cu.usbmodem1421',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

cur_th = [0,-50,-50,0,-50,0]
target_th = []
tmp_cmd = []
target_cmd = []
while 1 :
    # get keyboard input
    cmd = raw_input(">> ")
    if cmd == 'exit':
        ser.close()
        exit()
    else:
        #print "Wrote : %s" % (cmd)
        pose = map(float, cmd.split())
        if len(pose) != 9:
            usage()
            continue

        target_th.append(pose)
        for i in range (1,9):
            xp, yp, zp = [pose[0], round(pose[1]+RADIUS*math.sin(7.5*i*PI/180)*math.sin(7.5*i*PI/180)), round(pose[2]-RADIUS*math.sin(7.5*i*PI/180)*math.cos(7.5*i*PI/180))]
            #target_th.append([xp, yp, zp, xp+10, yp, zp, xp, yp, zp])#yp-10*(math.sin(7.5*i*PI/180)), zp-10*(1-math.cos(7.5*i*PI/180))])
            target_th.append([xp, yp, zp, xp+10, yp, zp, xp, yp+10*(math.sin(7.5*i*PI/180)), zp-10*(math.cos(7.5*i*PI/180))])

        for i in range(len(target_th)):
            if i == 0:
                time_span = 1
            else:
                time_span = 0.2
            ik_result = IK(cur_th, target_th[i], time_span)
            cur_th = list(ik_result)
            for j in range(6):
                tmp_cmd.append(str("%.2f"%(cur_th[j])))
            target_cmd.append(tmp_cmd)
            tmp_cmd = []

        cmd_input = "%i" % len(target_cmd)
        for i in range(len(target_cmd)):
            cmd_input += " ".join(target_cmd[i])
            cmd_input += " "
            print target_th[i],target_cmd[i]
            #print "cmd %i " % i, target_th[i], target_cmd[i]
        target_cmd = []
        target_th = []
        tmp_cmd = []
        print cmd_input
        
        ser.write(cmd_input)
        time.sleep(0.5)
        #ik_result = IK(cur_th, pose, 1)
        #cur_th = list(ik_result)
        #cmd_2 = ["","","","","",""]
        #for i in range(len(cmd_2)):
        #    cmd_2[i] = str(int(cur_th[i]))
        #
        #print "cmd input_1", " ".join(cmd_2)
        #print "cmd input_2", " ".join(cmd_1)
        
        #ser.write(" ".join(cmd_2)+" "+" ".join(cmd_1)+" ")
        #print cur_th
        #ser.write(' '.join(cur_th) + '\n')
        #print ' '.join(cur_th) + '\n'
    
        out = 'result'
        #while ser.inWaiting() > 0:
        #     out += ser.read(1)

        if out != '':
            print "<< " + out
