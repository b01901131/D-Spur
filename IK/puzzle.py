from FK import *
import serial
import math

PI = math.pi

def usage():
    print "Usage : input 9 pose parameters."


ser = serial.Serial(
    port='/dev/cu.usbmodem168',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)
ser.isOpen()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

cur_th = [0,-50,-50,0,-50,0]
key_loca = ["",""]
key_hole = ["",""]
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

        key_loca[0] = list(pose)
        key_loca[1] = list(pose)
        key_loca[0][2] = key_loca[0][2] + 50
        key_loca[0][5] = key_loca[0][5] + 50
        key_loca[0][8] = key_loca[0][8] + 50

        theta = 30
        key_loca[0][6] = key_loca[1][6] = key_loca[0][0] + 10*math.cos(theta*PI/180)
        key_loca[0][7] = key_loca[1][7] = key_loca[0][1] + 10*math.sin(theta*PI/180)
        
        pose[1] = pose[1] + 30
        pose[4] = pose[4] + 30
        pose[7] = pose[7] + 30
        key_hole[0] = list(pose)
        key_hole[1] = list(pose)
        key_hole[0][2] = key_hole[0][2] + 50
        key_hole[0][5] = key_hole[0][5] + 50
        key_hole[0][8] = key_hole[0][8] + 50

        
        for i in range(len(key_loca)):
            if i == 0:
                time_span = 1
            else:
                time_span = 0.5
            ik_result = IK(cur_th, key_loca[i], time_span)
            cur_th = list(ik_result)
            for j in range(6):
                tmp_cmd.append(str(int(cur_th[j])))
            target_cmd.append(tmp_cmd)
            tmp_cmd = []

        for i in range(len(key_hole)):
            if i == 0:
                time_span = 1
            else:
                time_span = 0.5
            ik_result = IK(cur_th, key_hole[i], time_span)
            cur_th = list(ik_result)
            for j in range(6):
                tmp_cmd.append(str(int(cur_th[j])))
            target_cmd.append(tmp_cmd)
            tmp_cmd = []
        cmd_input = "%i" % len(target_cmd)
        for i in range(len(target_cmd)):
            cmd_input += " ".join(target_cmd[i])
            cmd_input += " "
            print target_cmd[i]
            #print "cmd %i " % i, target_th[i], target_cmd[i]
        target_cmd = []
        tmp_cmd = []
        print cmd_input
        ser.write(cmd_input)
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
