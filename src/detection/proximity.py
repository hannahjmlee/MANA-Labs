from src.utils.distance_metrics import EuclideanDistance

class Proximity: 
    # Constructor: Initializes the Proximity instance with a proximity limit.
    def __init__(self, limit=2.05):
        self.limit = max(limit, 2.05)  # Ensures the limit is at least 2.05

    # Method: Checks if the distance between two positions is within a specified range.
    def InRange(self, posA, posB, limit=None): 
        if limit is None:  # Use the instance's limit if no limit is provided
            return self.InProximity(posA, posB, self.limit)
        return self.InProximity(posA, posB, limit)

    # Method: Determines if two positions are within the specified proximity limit.
    def InProximity(self, posA, posB, limit): 
        if EuclideanDistance(posA, posB) <= limit:  # Check if distance is within the limit
            return True
        return False
