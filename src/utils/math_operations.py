from math import sqrt 

def RangeOverlap(rangeA, rangeB):
    Ax, Ay = rangeA
    Bx, By = rangeB

    if (Ax[0] > Bx[1] or Ax[1] < Bx[0]) and (Ay[0] > By[1] or Ay[1] < By[0]):
        return True
    return False

def GetDirectionVector(posA, posB, normalize=True): 
    x1, y1 = posA
    x2, y2 = posB

    vx = x2 - x1
    vy = y2 - y1

    if normalize: 
        mag = sqrt(vx**2 + vy**2)
        vx = vx / mag
        vy = vy / mag

    return (vx, vy)

def GetSlopeIntercept(posA, posB): 
    x1, y1 = posA
    x2, y2 = posB

    if x1 == x2: 
        return None, None

    slope = (y1 - y2) / (x1 - x2)
    intercept = y1 - slope * x1

    return slope, intercept 

