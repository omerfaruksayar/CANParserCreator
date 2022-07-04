#Program takes 6 parameters.
#Usage: python3 canparsercreator.py <package name> <dbc path> <package path> <package message name> <subscribing topic name for can messages> <publisher topic name>

import sys
from unittest import signals

from numpy import size

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
            
        for counter in range(size(signalGroups)):
            
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
                        
if __name__ == '__main__':
    db_name = sys.argv[1]
    dbc_path = sys.argv[2]
    pckg_path = sys.argv[3]
    msg_name = sys.argv[4]
    sbs_topic = sys.argv[5]
    pbs_topic = sys.argv[6]
    structs = readHeaderFile(pckg_path+"/"+db_name+"/include/"+db_name+".h")
    for s in structs:
        s.printSGC()