import math
import numpy 

def cosine_rule_distance(dist_1,dist_2,angle):
    distance = math.sqrt((dist_1**2) + (dist_2**2) - 2*(dist_1*dist_2*math.cos(math.radians(angle))))
    return distance



def FuncLx(x,y,Z):    
        Lx = numpy.zeros((2,6))

        Lx[0,0] = -1/Z
        Lx[0,1] = 0
        Lx[0,2] = x/Z
        Lx[0,3] = x*y
        Lx[0,4] = -(1+x**2)
        Lx[0,5] = y

        Lx[1,0] = 0
        Lx[1,1] = -1/Z
        Lx[1,2] = y/Z
        Lx[1,3] = 1+y**2
        Lx[1,4] = -x*y
        Lx[1,5] = -x

        return Lx



def left_or_right(cam_center_x,marker_centroid_x):
    tolerance = 10
    result = None
    
    if marker_centroid_x < cam_center_x - tolerance:
        result = 'left'
    elif marker_centroid_x > cam_center_x + tolerance:
        result = 'right'
    else:
        result = 'straight'

    return result