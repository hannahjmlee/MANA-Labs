# Function: Checks for potential collisions between two moving entities on a grid.
# Returns a tuple indicating whether a collision occurred and the type of constraint (edge or vertex).
def GridCollisionCheck(posA0, posA1, posB0, posB1, t0, t1):
    # Check for edge constraint: If the positions swap between A and B
    if posA0 == posB1 and posA1 == posB0:
        return True, (posA0, posA1), (t0, t1)  
    # Check for vertex constraint: If both A and B end up at the same position
    elif posA1 == posB1:
        return True, posA1, t1  
    return False, None, None  # No collision detected
