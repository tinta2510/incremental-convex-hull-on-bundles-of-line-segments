import math

class Point:
    def __init__(self, x: float, y: float) -> None: 
        self.x = x
        self.y = y
    
    def __repr__(self) -> str:
        return f"({self.x}, {self.y})"
    
    def __eq__(self, other):
        # Check if 'other' is an instance of Point
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))
    
    @property
    def magnitude(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def __add__(self, other: 'Point') -> 'Point':
        return Point(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar: float) -> 'Point':
        return Point(self.x * scalar, self.y * scalar)
    
    def __truediv__(self, scalar: float) -> 'Point':
        return Point(self.x / scalar, self.y / scalar)

    def __ne__(self, other):
        # Negate the result of __eq__
        return not self.__eq__(other)

EPSILON = 1e-6

def calculate_distance(point1: Point, point2: Point) -> float:
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2) 

def is_larger_angle(angle1: float, angle2: float) -> bool:
    return angle1 - angle2 > EPSILON

def is_equal_angle(angle1: float, angle2: float) -> bool:
    return abs(angle1 - angle2) <= EPSILON

def is_smaller_angle(angle1: float, angle2: float) -> bool:
    return angle1 - angle2 < -EPSILON

def is_left(p0: Point, p1: Point, p: Point, direction: bool = True) -> bool:
    return (((p1.x - p0.x)*(p.y - p0.y) - (p.x - p0.x)*(p1.y - p0.y)) * 
            (1 if direction else -1)) > EPSILON
    
def is_left_on(p0: Point, p1: Point, p: Point, direction: bool = True) -> bool:
    return (((p1.x - p0.x)*(p.y - p0.y) - (p.x - p0.x)*(p1.y - p0.y)) * 
            (1 if direction else -1)) >= -EPSILON

def calculate_angle(A: Point, B: Point, C: Point) -> float:
    """
    Calculate the angle ABC formed by three points A, B, and C (in order).

    Args:
        A (Point): The first point.
        B (Point): The second point (vertex).
        C (Point): The third point.

    Returns:
        float: The angle in degrees.
    """
    # Vectors AB and BC
    BA_x = A.x - B.x
    BA_y = A.y - B.y
    BC_x = C.x - B.x
    BC_y = C.y - B.y

    # Dot product
    dot_product = BA_x * BC_x + BA_y * BC_y
    
    # Magnitudes
    magnitude_AB = math.sqrt(BA_x**2 + BA_y**2)
    magnitude_BC = math.sqrt(BC_x**2 + BC_y**2)

    # Avoid division by zero
    if magnitude_AB == 0 or magnitude_BC == 0:
        raise ValueError("One of the vectors has zero length.")
    
    # Cosine of the angle
    cos_theta = dot_product / (magnitude_AB * magnitude_BC)
    
    # Ensure the value is within the valid range for acos
    cos_theta = max(-1, min(1, cos_theta))

    # Calculate angle in radians and convert to degrees
    angle_radians = math.acos(cos_theta)
    angle_degrees = math.degrees(angle_radians)
    
    return angle_degrees
            
def orientation(p: Point, q: Point, r: Point) -> int:
    """
    Determines the orientation of the triplet (p, q, r).
    Returns:
    0 -> Collinear
    1 -> Clockwise
    -1 -> Counterclockwise
    """
    val = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)
    if val == 0:
        return 0  # Collinear
    return 1 if val > 0 else -1  # Clockwise or Counterclockwise

def on_segment(p: Point, q: Point, r: Point) -> bool:
    """
    Checks if point r lies on segment pq (assuming collinear condition is met).
    """
    return min(p.x, q.x) + EPSILON <= r.x <= max(p.x, q.x) + EPSILON and min(p.y, q.y) + EPSILON <= r.y <= max(p.y, q.y) + EPSILON

def do_intersect(A: Point, B: Point, C: Point, D: Point) -> bool:
    """
    Returns True if segments AB and CD intersect.
    """
    o1 = orientation(A, B, C)
    o2 = orientation(A, B, D)
    o3 = orientation(C, D, A)
    o4 = orientation(C, D, B)

    # General case: opposite orientations
    if o1 != o2 and o3 != o4:
        return True

    # Special case: Check collinear points and if they overlap
    if o1 == 0 and on_segment(A, B, C): return True
    if o2 == 0 and on_segment(A, B, D): return True
    if o3 == 0 and on_segment(C, D, A): return True
    if o4 == 0 and on_segment(C, D, B): return True

    return False

class ConvexHull:
    @staticmethod
    def incremental_convex_hull(points: list[Point]) -> list[Point]:
        """
        Computes the convex hull of a set of points using the monotone chain algorithm (Andrew's algorithm).
        Returns the vertices of the convex hull in counterclockwise order.
        Time complexity: O(n log n)
        """
        if len(points) < 3:
            return points  # Convex hull is not defined for less than 3 points

        # Remove duplicate points
        unique_points = list(set(points))
        if len(unique_points) < 3:
            return unique_points

        # Sort points lexicographically (by x, then by y)
        sorted_points = sorted(unique_points, key=lambda p: (p.x, p.y))
        
        # Build lower hull
        lower = []
        for p in sorted_points:
            # Remove points that make a clockwise turn (or collinear)
            while len(lower) >= 2 and orientation(lower[-2], lower[-1], p) != -1:
                lower.pop()
            lower.append(p)
        
        # Build upper hull
        upper = []
        for p in reversed(sorted_points):
            # Remove points that make a clockwise turn (or collinear)
            while len(upper) >= 2 and orientation(upper[-2], upper[-1], p) != -1:
                upper.pop()
            upper.append(p)
        
        # Remove the last point of each half because it's repeated
        # The last point of lower is the first point of upper and vice versa
        lower.pop()
        upper.pop()
        
        # Combine lower and upper hull to get the complete convex hull
        # Return in counterclockwise order
        return lower + upper

    @staticmethod
    def extract_convex_rope_from_hull(hull: list[Point], start_pt: Point, end_pt: Point, clockwise: bool = True) -> list[Point]:
        """
        Extracts a convex rope from the convex hull between start_pt and end_pt.
        The rope is extracted in clockwise or counterclockwise order based on the 'clockwise' parameter.
        """
        if start_pt not in hull or end_pt not in hull:
            raise ValueError("Start and end points must be part of the convex hull.")

        start_index = hull.index(start_pt)
        end_index = hull.index(end_pt)

        if clockwise:
            if start_index <= end_index:
                return hull[start_index:end_index + 1]
            else:
                return hull[start_index:] + hull[:end_index + 1]
        else:
            if start_index >= end_index:
                return hull[end_index:start_index + 1][::-1]
            else:
                return list(reversed(hull[end_index:] + hull[:start_index + 1]))
            
    @staticmethod
    def find_tangent(poly1: list[Point], poly2: list[Point]) -> tuple[Point, Point]:
        """
        Finds the tangent between two convex hulls (assumes clockwise orientation).
        Connects the leftmost possible point of poly1 to the rightmost possible point of poly2.
        Returns the points of tangency as (point_on_poly1, point_on_poly2).
        """
        if len(poly1) < 2 or len(poly2) < 2:
            raise ValueError("Both polygons must have at least 2 points")
        
        def next_index(i: int, n: int) -> int:
            return (i + 1) % n
        
        def prev_index(i: int, n: int) -> int:
            return (i - 1) % n
        
        def find_leftmost_point(poly: list[Point]) -> int:
            """Find the index of the leftmost point (min x-coordinate)"""
            return min(range(len(poly)), key=lambda i: poly[i].x)
        
        def find_rightmost_point(poly: list[Point]) -> int:
            """Find the index of the rightmost point (max x-coordinate)"""
            return max(range(len(poly)), key=lambda i: poly[i].x)
        
        n1, n2 = len(poly1), len(poly2)
        
        # Start with leftmost point of poly1 and rightmost point of poly2
        i = find_leftmost_point(poly1)
        j = find_rightmost_point(poly2)
        
        # Iteratively adjust to find the tangent
        while True:
            changed = False
            
            # For poly1 (clockwise): check adjacent points
            prev_i = prev_index(i, n1)
            next_i = next_index(i, n1)
            
            prev_j = prev_index(j, n2)
            next_j = next_index(j, n2)

            
            # External tangent: both polygons on same side of tangent line
            # For clockwise poly1, move to maintain all points on right side
            if (orientation(poly1[i], poly2[j], poly1[prev_i]) 
                    != orientation(poly1[i], poly2[j], poly1[next_i]) 
                or orientation(poly1[prev_i], poly1[i], poly2[j])
                    != orientation(poly1[prev_i], poly1[i], poly1[next_i])
            ):  # prev point on left (wrong side)
                i = next_i
                changed = True
            
            # For poly2 (clockwise): check adjacent points


            if (orientation(poly1[i], poly2[j], poly2[prev_j]) 
                    != orientation(poly1[i], poly2[j], poly2[next_j])
                or orientation(poly1[i], poly2[j], poly2[next_j])
                    != orientation(poly2[prev_j], poly2[j], poly2[next_j])
            ):  # prev point on left (wrong side)
                j = prev_j
                changed = True
                
            # If no changes were made, we found the tangent
            if not changed:                
                break
        
        return (poly1[i], poly2[j])