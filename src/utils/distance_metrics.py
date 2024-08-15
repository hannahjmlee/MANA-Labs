
def ManhattanDistance(posA, posB):
    x1, y1 = posA
    x2, y2 = posB

    return abs(x1 - x2) + abs(y1 - y2)

def EuclideanDistance(posA, posB):
    x1, y1 = posA
    x2, y2 = posB

    return ((x1 - x2)**2 + (y1-y2)**2)**0.5
