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
    
    def __hash__(self):
        # Combine the hash of x and y to create a unique hash for each Point
        return hash((self.x, self.y))

    def __ne__(self, other):
        # Negate the result of __eq__
        return not self.__eq__(other)
    
class LineSegment:
    def __init__(self, start_point: Point, end_point: Point) -> None:
        self.start_point = start_point
        self.end_point = end_point
    
    def __repr__(self) -> str:
        return f"[{self.start_point, self.end_point}]"
    
    @property
    def length(self) -> float:
        return calculate_distance(self.start_point, self.end_point)

EPSILON = 1e-6

def calculate_distance(point1: Point, point2: Point) -> float:
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2) 

def is_larger_angle(angle1: float, angle2: float) -> bool:
    return angle1 - angle2 > EPSILON

def is_equal_angle(angle1: float, angle2: float) -> bool:
    return abs(angle1 - angle2) <= EPSILON

def is_smaller_angle(angle1: float, angle2: float) -> bool:
    return angle1 - angle2 < -EPSILON

def is_left(p0: Point, p1: Point, p: Point, direction: bool = 1) -> bool:
    return (((p1.x - p0.x)*(p.y - p0.y) - (p.x - p0.x)*(p1.y - p0.y)) * 
            (1 if direction else -1)) > EPSILON
    
def is_left_on(p0: Point, p1: Point, p: Point, direction: bool) -> bool:
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
            