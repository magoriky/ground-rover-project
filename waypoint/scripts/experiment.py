from pyproj import Proj
from numpy import sqrt
p = Proj(proj='utm', zone=52,ellps='WGS84',preserve_units=False)
def convert2Utm(lat,long):
    x,y = p(lat,long)
    return x,y

def convert2LongLat(x,y):
    long,lat = p(x,y, inverse=True)
    return long,lat

def distance_between_points(lat1,long1,lat2,long2):
    x1, y1 = convert2Utm(lat1,long1)
    x2, y2 = convert2Utm(lat2,long2)
    distance = sqrt((x1-x2)**2 + (y1-y2)**2)
    return distance

def middlePoint(lat1,long1,lat2,long2):
    x1, y1 = convert2Utm(lat1,long1)
    x2, y2 = convert2Utm(lat2,long2)
    m = (0.5*(x1+x2),0.5*(y1+y2))
    long, lat = convert2LongLat(m[0],m[1])
    return lat, long

def getMiddlePoints(pointsList):
    newList = []
    for i in range(len(pointsList)-1):
        p1 = pointsList[i]
        p2 = pointsList[i+1]
        lat, long = middlePoint(p1[0],p1[1],p2[0],p2[1])
        newList.extend([(p1[0],p1[1]),(lat,long),(p2[0],p2[1])])
    return newList

def divideSeveralPoints(pointsList,numPoints):
    for i in range(numPoints):
        pointsList = getMiddlePoints(pointsList)
    newList = pointsList
    return newList
        
pointsList = [(37.373552,126.666995), (37.373682,126.667118)]
newList = divideSeveralPoints(pointsList,1)
print(newList)       
        
    
    
    



