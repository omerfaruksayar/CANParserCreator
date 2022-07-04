#Program takes 6 parameters.
#Usage: python3 canparsercreator.py <package name> <dbc path> <package path> <package message name> <subscribing topic name for can messages> <publisher topic name>

import sys

class SignalGroupStruct:
    signals = []
    def __init__(self,name,id):
        self.name = name
        self.frameId = id
        
    def printSGC(self):
        print(self.name,self.frameId)    
    
#Reads the frame ids int he header file and returns SignalGroupStruct lists with empty signals.
def readFrameIds(filename):
    frameIds = []
    with open(filename) as file:
        for line in file:
            if '/* Frame ids. */' in line:
                break    
        for line_1 in file:
            if '/* Frame lengths in bytes. */' in line_1:
                break 
              
            frameIds.append(line_1.rstrip('\n'))
            
    frameIds.pop()
    signalGroups = []

    for f in frameIds:
        signalGroups.append(SignalGroupStruct(f.split()[1].lower().replace("frame_id", "t"), f.split()[2].replace("(","").replace(")","")))
    
    return signalGroups   
                        
if __name__ == '__main__':
    db_name = sys.argv[1]
    dbc_path = sys.argv[2]
    pckg_path = sys.argv[3]
    msg_name = sys.argv[4]
    sbs_topic = sys.argv[5]
    pbs_topic = sys.argv[6]
    structs = readFrameIds(pckg_path+"/"+db_name+"/include/"+db_name+".h")
    for s in structs:
        s.printSGC()