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
    def __init__(self,name,id):
        self.name = name
        self.frameId = id
        
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
            signalGroups.append(SignalGroupStruct(f.split()[1].lower().replace("frame_id", "t"), f.split()[2].replace("(","").replace(")","")))
            
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
def fiilMessage(structs,msgPath,dbcPath):
    os.system("iconv -f windows-1252 -t utf-8 " + dbc_path + " > " + dbc_path+".txt")
    vals = []
    with open(dbcPath+".txt") as dbc:
        for line in dbc:
            if 'VAL_ ' in line:
                vals.append(line.strip().split()[2].lower())               
          
    with open(msgPath, 'w') as mf:
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
    
    os.system("rm -rf " + dbc_path+".txt")                
                                                        
if __name__ == '__main__':
    db_name = sys.argv[1]
    dbc_path = sys.argv[2]
    pckg_path = sys.argv[3]
    msg_name = sys.argv[4]
    sbs_topic = sys.argv[5]
    pbs_topic = sys.argv[6]
    structs = readHeaderFile(pckg_path+'/'+db_name+'/include/'+db_name+'.h')
    fiilMessage(structs,pckg_path+'/'+db_name+'/msg/'+msg_name+'.msg',dbc_path)