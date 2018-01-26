import socket
import configparser

#Read the config file for anchor positions
config = configparser.ConfigParser()
config.read('config.ini')
anchorNames = config.options('anchorpos')
anchorCoords = {anchorName: None for anchorName in anchorNames}
tagIDs = config.options('tagnames')
for anchorName in anchorNames:
    coords = config['anchorpos'][anchorName].split(',')
    anchorCoords[anchorName] = {'x':float(coords[0]),'y':float(coords[1]),'z':float(coords[2])}

#Read the config file for the number of cycles to keep data
historyLength = int(config['default']['historyLength'])

#Read the config file for the maximum error when doing calculations
epsilon = float(config['default']['epsilon'])

#Create a list of 2D dictionaries to store distances reported from each anchor\tag pair
data = [{tagID: {anchorID: None for anchorID in anchorNames} for tagID in tagIDs}] * historyLength

numAnchors = len(anchorNames)

#Bind to UDP port 10000 to capture the tag's traffic
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind(('', int(config['default']['port'])))
lastTimestamp = {tagName: 0 for tagName in tagIDs}
numAnchorsReported = {tagName: 0 for tagName in tagIDs}
while True:
    data, addr = sock.recvfrom(100)
    print("received message:"+data.decode("ascii"))
    
    dataArr = data.decode("ascii").split(",")
    if len(dataArr) == 5 and dataArr[0] in anchorNames and dataArr[1] in tagIDs:
        #Give array of results human readable names
        anchorID = dataArr[0]
        tagID = dataArr[1]
        dist = dataArr[2]
        timestamp = dataArr[3]
        tagPower = dataArr[4]
        #TODO: verify inputs to prevent crashing
        data[timestamp % historyLength][tagID][anchorID] = float(dist)
        
        #Keep track of whether or not we can do calculations
        if lastTimeStamp[tagID] == timestamp:
            numAnchorsReported[tagID] += 1
        else:
            lastTimeStamp[tagID] = timestamp
            numAnchorsReported[tagID] = 0
        #Each anchor responded with a distance
        if NumAnchorsReported[tagID] == numAnchors:
            sumDist = 0.0
            for anchorName in anchorNames:
                sumDist += 1.0/data[timestamp % historyLength][tagID][anchorName]
            x = 0.0
            y = 0.0
            for anchorName in anchorNames:
                x += 1.0/(anchorCoords[anchorName]['x']*(data[timestamp % historyLength][tagID][anchorName])/sumDist)
                y += 1.0/(anchorCoords[anchorName]['y']*(data[timestamp % historyLength][tagID][anchorName])/sumDist)
            
            print("Located Tag "+tagID+" at time "+timestamp+"at ("+x+","+y+")")