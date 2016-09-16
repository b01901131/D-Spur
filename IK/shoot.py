from FK import *
import serial
import math

def usage():
    print "Usage : input 3 pose parameters."


ser = serial.Serial(
    port='/dev/cu.usbmodem1421',
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
        pose = map(str, cmd.split())
        if len(pose) != 3:
            usage()
            continue

        
        

        cmd_input = "1"
        cmd_input += " ".join(pose)
        cmd_input += " "
        print cmd_input
        pose = []
        
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
