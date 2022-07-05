#Program takes 6 parameters, 
#package name = Name of the package will be created in the given WS path
#dbc path = path for dbc file containing .dbc file
#package path = Path for WS/src
#package message name = Message file name will be created in the package
#subscribing topic name = Topic for can data
#publisher topic name = Topic name to publish our message
#Usage: python3 canparsercreator.py <package name> <dbc path> <package path> <package message name> <subscribing topic name for can messages> <publisher topic name>

import sys
import os

class SignalGroupStruct:
    signals = []
    def __init__(self,name,id,header):
        self.name = name
        self.frameId = id
        self.header = header
        
    def printSGC(self):
        print(self.name,self.frameId)
        print("Signals: ")
        for s in self.signals:
            print(s)   
    
#Reads the header file into a SignalGroupStruct list and returns the list.
def readHeaderFile(filename):
    frameIds = []
    signals = []
    signalGroups = []
    
    with open(filename) as file:
        for line in file:
            if '/* Frame ids. */' in line:
                break    
        for line_1 in file:
            if '/* Frame lengths in bytes. */' in line_1:
                break 
            frameIds.append(line_1.rstrip('\n'))
        
        frameIds.pop()
        for f in frameIds:
            signalGroups.append(SignalGroupStruct(f.split()[1].lower().replace("frame_id", "t"), f.split()[2].replace("(","").replace(")",""),f.split()[1]))
            
        for counter in range(len(signalGroups)):
            
            for line_2 in file:
                if 'struct' in line_2:
                    break
                
            for line_3 in file:
                if '};' in line_3:
                    break
                
                if 'int' in line_3:
                    signals.append(line_3.strip().split()[1].replace(";",""))
                    
            signalGroups[counter].signals = signals
            signals = []    
                               
    return signalGroups   

#Converts the dbc encoding to utf-8 and determines the types in the message according to dbc and writes to the message file.                                 
def fiilMessage(structs,msg_path,dbc_path):
    os.system("iconv -f windows-1252 -t utf-8 " + dbc_path + " > " + dbc_path+".txt")
    vals = []
    with open(dbc_path+".txt") as dbc:
        for line in dbc:
            if 'VAL_ ' in line:
                vals.append(line.strip().split()[2].lower())               
   
    with open(msg_path, 'w') as mf:
        mf.write("std_msgs/Header header\n")
        mf.write("\n")
        for struct in structs:
            mf.write("# %s "% struct.name.upper())
            mf.write("%s\n" % struct.frameId)
            for signal in struct.signals:
                if (signal in vals):
                    mf.write("uint64 %s\n" % signal)              
                else:
                    mf.write("float64 %s\n" % signal)
            mf.write("\n")
    
    os.system("rm -rf " +dbc_path+".txt")                
                                    
def writeCpp(structs,srcPath,msgName,dbName,sbsTopic,pbsTopic):
    headers = '#include <ros/ros.h>\n#include "can_msgs/Frame.h"\n#include "'+dbName+'.h"\n#include "'+msgName+'.h"\nusing namespace std;'
    classPublic = 'class '+dbName.upper()+'Feedback{\n\tpublic:\n\t\t'+dbName.upper()+'Feedback(){\n\t\t\tros::NodeHandle private_nh;\n\t\t\tcan_sub = private_nh.subscribe("'+sbsTopic+'", 1000, &'+dbName.upper()+'Feedback::canCallback, this);\n\t\t\t'+dbName+'_publisher = private_nh.advertise<'+msgName+'>("'+dbName.upper()+'_feedback", 1000);\n\t\t}\n\t\t~'+dbName.upper()+'Feedback(){}'
    classPrivate = '\n\tprivate:\n\t\tros::Publisher '+dbName+'_publisher;\n\t\tros::Subscriber can_sub;\n'
    for struct in structs:
        pName = ''.join(struct.name.split('_')[1:len(struct.name.split('_'))-1])
        classPrivate += '\t\t'+struct.name+' *'+pName+' = new '+struct.name+';\n' 
    
    callBack = '\n\t\tvoid canCallback(const can_msgs::Frame msg){\n\t\t\t'+msgName+' '+msgName.lower()+'_msg;\n\t\t\tuint id = (msg.id > 2147483647) ? msg.id ^ 0x80000000 : msg.id;\n\t\t\tswitch(id){\n'
    
    for struct in structs:
        pName = ''.join(struct.name.split('_')[1:len(struct.name.split('_'))-1])
        callBack += '\t\t\tcase ' +struct.header + ':\n'
        callBack += '\t\t\t\t'+struct.header.lower().replace("_frame_id","_unpack")+'('+ pName + ',msg.data,data(),8);\n'
        for signal in struct.signals:
            callBack += '\t\t\t\t'
      
    with open(srcPath, 'w') as cpp:
        cpp.write(headers+'\n\n')
        cpp.write(classPublic+'\n')
        cpp.write(classPrivate)
        cpp.write(callBack)                                   
                                                        
if __name__ == '__main__':
    db_name = sys.argv[1]
    dbc_path = sys.argv[2]
    pckg_path = sys.argv[3]
    msg_name = sys.argv[4]
    sbs_topic = sys.argv[5]
    pbs_topic = sys.argv[6]
    os.system("./generateParser.sh " + db_name + " " + dbc_path + " " + pckg_path + " " + msg_name + " " + sbs_topic + " " + pbs_topic)
    structs = readHeaderFile(pckg_path+'/'+db_name+'/include/'+db_name+'.h')
    fiilMessage(structs,pckg_path+'/'+db_name+'/msg/'+msg_name+'.msg',dbc_path)
    writeCpp(structs,pckg_path+'/'+db_name+'/src/parser.cpp',msg_name,db_name,sbs_topic,pbs_topic)