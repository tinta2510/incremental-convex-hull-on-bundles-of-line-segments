import logging
from itertools import combinations
from utils import calculate_distance, is_larger_angle, \
                    is_equal_angle, \
                    is_left, is_left_on, calculate_angle, \
                    Point, do_intersect

LOG_FILE = "./convex_hull.log"  
with open(LOG_FILE, 'w') as file:  # Open in write mode to clear content
    pass   
                   
def write_points_to_file(points: list[Point], filename: str) -> None:
    """
    Write a list of points to a file. (For logging purpose)

    :param points: List of Point objects.
    :param filename: Name of the file to write the points to.
    """
    try:
        with open(filename, 'a') as file:
            for point in points:
                # Format the point for writing to the file
                file.write(f"{point.x} {point.y}\n")
            file.write("\n")
    except Exception as e:
        print(f"An error occurred while writing points to the file: {e}")

class SequenceOfBundles:
    """
    Class for representing a sequence of bundles
    
    Attributes:
        skeleton (list[Point]): the skeleton (a sequence of vertices)
        radius (float): the max length of line segment in bundles 
        outer_endpoints (list[list[Point]]): list of list of outer endpoinst of line segments
    """
    def __init__(self, skeleton: list[Point], radius: float) -> None:
        self.skeleton = skeleton
        self.radius = radius
        self.outer_endpoints: list[list[Point]] = [[] for _ in range(len(self.skeleton))]
        
    def add_line_segment(self, vertex: Point, outer_endpoint: Point) -> None:
        if not vertex in self.skeleton:
            logging.error(f"Vertex {vertex} not in the skeleton {self.skeleton}")
            return        
        if vertex in [self.skeleton[0], self.skeleton[-1]]:
            logging.error(f"Cannot add line segments [{vertex}, {outer_endpoint}] into the first and last bundles.")
            return
        if calculate_distance(vertex, outer_endpoint) > self.radius:
            diff = outer_endpoint - vertex               
            direction = diff / diff.magnitude
            outer_endpoint = vertex + direction * self.radius
            logging.error(f"The length of the added segment [{vertex}, {outer_endpoint}] is greater than the max radius. Preprocessed!")
        
        index = self.skeleton.index(vertex)
        prev_vertex = self.skeleton[index-1]
        next_vertex = self.skeleton[index+1]
        if not is_equal_angle(
            calculate_angle(prev_vertex, vertex, next_vertex),
            calculate_angle(prev_vertex, vertex, outer_endpoint) +
                calculate_angle(outer_endpoint, vertex, next_vertex)
        ): # ensure the added segments lies in the sector that smaller than pi
            logging.error(f"The segment [{vertex}, {outer_endpoint}] must lie in the sector that smaller than pi")
            return
        
        if self.outer_endpoints[index] == []: 
            self.outer_endpoints[index].append(outer_endpoint)
        else:
            for i, p in enumerate(self.outer_endpoints[index]):
                angle1 = calculate_angle(prev_vertex, vertex, p)
                angle2 = calculate_angle(prev_vertex, vertex, outer_endpoint)
                if is_larger_angle(angle1, angle2):
                    self.outer_endpoints[index].insert(i, outer_endpoint) 
                    return
                elif is_equal_angle(angle1, angle2):
                    return
            self.outer_endpoints[index].append(outer_endpoint)     
    
    def preprocess(self):
        min_radius = 0.5 * min(calculate_distance(a, b) for a, b in combinations(self.skeleton, 2))
        for i, vertex in enumerate(self.skeleton):
            for j, outer_endpoint in enumerate(self.outer_endpoints[i]):
                if calculate_distance(vertex, outer_endpoint) > min_radius:
                    diff = outer_endpoint - vertex               
                    direction = diff / diff.magnitude
                    self.outer_endpoints[i][j] = vertex + direction * min_radius

    @staticmethod
    def load_sequence_from_file(filename: str, preprocess: bool = True):
        with open(filename, 'r') as file:
            lines = file.readlines()
        
        radius = None
        vertices = []
        line_segments = []
        section = None  # Keep track of the current section
        
        for line in lines:
            line = line.strip()
            if not line or line.startswith("#"):
                continue  # Skip empty lines and comments
            
            # Detect section headers
            if line.startswith("Radius:"):
                section = "Radius"
                radius = float(line.split(":")[1].strip())
            elif line.startswith("Vertices:"):
                section = "Vertices"
            elif line.startswith("LineSegments:"):
                section = "LineSegments"
            else:
                # Parse data based on the current section
                if section == "Vertices":
                    x, y = map(float, line.split())
                    vertices.append(Point(x, y))
                elif section == "LineSegments":
                    data = list(map(float, line.split()))
                    vertex_index = int(data[0])
                    outer_endpoint = Point(data[1], data[2])
                    line_segments.append((vertex_index, outer_endpoint))
        
        # Validate required data
        if radius is None or not vertices:
            raise ValueError("Input file is missing required data: Radius or Vertices.")
        
        # Initialize the sequence
        sequence = SequenceOfBundles(vertices, radius)
        
        # Add line segments to the sequence
        for vertex_index, outer_endpoint in line_segments:
            if 0 <= vertex_index < len(vertices):
                sequence.add_line_segment(vertices[vertex_index], outer_endpoint)
            else:
                logging.error(f"Invalid vertex index {vertex_index} for line segment {outer_endpoint}")
        
        if preprocess: sequence.preprocess() # Preprocess the sequence
        return sequence
            
class SimplePolygon:
    def __init__(self, polyline_P: list[Point], polyline_Q: list[Point]):
        if (polyline_P[0] != polyline_Q[0] or 
            polyline_P[-1] != polyline_Q[-1]
        ):
            raise ValueError("Start and end point of two polylines must be the same.")
        self.polyline_P = polyline_P
        self.polyline_Q = polyline_Q
        # Future work: check intersection
        
    def is_inside_new_hull(self, left_tp: Point, right_tp: Point, added_pt: Point, checking_pt: Point, direction: bool):
        """
        Check if the dual polyline intersects with the newly extended convex hull
        """
        return (is_left(left_tp, added_pt, checking_pt, direction) and 
                is_left(added_pt, right_tp, checking_pt, direction) and 
                is_left(right_tp, left_tp, checking_pt, direction))
    
    def verify_link(self,
                   Xstar: list[Point], 
                   Ystar: list[Point], 
                   Ustar: Point, 
                   Vstar: Point, 
                   direction: bool
    ):
        for pt in Xstar:
            if not is_left_on(Ustar, Vstar, pt, direction):
                return False
        for pt in Ystar:
            if not is_left_on(Ustar, Vstar, pt, not direction):
                return False
        return True
    
    def find_link(self, Xstar, Ystar, direction):
        for Ustar in Xstar:
            for Vstar in Ystar:
                if self.verify_link(Xstar, Ystar, Ustar, Vstar, direction):
                    return Ustar, Vstar
        return None, None

    def get_tangent_line(self, tangent_polyline: list[Point], start_tp_idx: int, end_tp_idx: int):
        if end_tp_idx < start_tp_idx:
            return tangent_polyline[start_tp_idx:] + tangent_polyline[1:end_tp_idx+1]
        else: 
            return tangent_polyline[start_tp_idx:end_tp_idx+1]
        
    def find_tangent_points(self, tangent_polyline, added_pt, direction):
        left_tp_idx = len(tangent_polyline) - 1
        while left_tp_idx > 0 and not is_left(
                tangent_polyline[left_tp_idx-1], 
                tangent_polyline[left_tp_idx], 
                added_pt, 
                direction
            ):
            left_tp_idx = left_tp_idx - 1
        right_tp_idx = 0
        while right_tp_idx < len(tangent_polyline)-1 and not is_left(
                added_pt, 
                tangent_polyline[right_tp_idx], 
                tangent_polyline[right_tp_idx+1], 
                direction
            ):
            right_tp_idx = right_tp_idx + 1
        return left_tp_idx, right_tp_idx

    def find_left_tangent_point(self, tangent_polyline, added_pt, direction):
        left_tp_idx = len(tangent_polyline) - 1
        while left_tp_idx > 0 and not is_left(
                tangent_polyline[left_tp_idx-1], 
                tangent_polyline[left_tp_idx], 
                added_pt, 
                direction
            ):
            left_tp_idx = left_tp_idx - 1
        return left_tp_idx
        
    def find_shortest_path(self, direction: bool =  True):
        shortest_path = []
        start_index = 0
        checking_pt_index = 0
        curr_polyline = self.polyline_P if direction else self.polyline_Q
        while True:
            dual_polyline = self.polyline_Q  \
                             if curr_polyline == self.polyline_P \
                             else self.polyline_P
            tangent_polyline = []                 

            if curr_polyline[start_index] == self.polyline_P[-1]:
                shortest_path += [curr_polyline[start_index]]
                return shortest_path
            elif curr_polyline[start_index+1] == self.polyline_P[-1]:
                shortest_path += [curr_polyline[start_index], curr_polyline[start_index+1]]
                return shortest_path
            
            # Initialize the tangent polyline
            tangent_polyline.append(curr_polyline[start_index])
            tangent_polyline.append(curr_polyline[start_index+1])

            # Increment the convex hull
            for i in range(start_index+2, len(curr_polyline)):
                added_pt = curr_polyline[i]

                # Find the tangent points
                left_tp_idx = self.find_left_tangent_point(tangent_polyline, added_pt, direction)
                
                # # Check intersection
                Ystar = []
                intersection = False
                for prev_pt, pt in zip(dual_polyline[checking_pt_index:], dual_polyline[checking_pt_index+1:]):
                    if (not is_left(prev_pt, tangent_polyline[left_tp_idx], added_pt, direction) and
                        is_left(pt, tangent_polyline[left_tp_idx], added_pt, direction) and
                        do_intersect(prev_pt, pt, tangent_polyline[left_tp_idx], added_pt) 
                    ):
                        intersection = True
                        Ystar.append(pt)
                    elif (is_left(pt, tangent_polyline[left_tp_idx], added_pt, direction) and intersection):
                        Ystar.append(pt)
                    elif (not is_left(pt, tangent_polyline[left_tp_idx], added_pt, direction) and
                        do_intersect(prev_pt, pt, tangent_polyline[left_tp_idx], added_pt) ):
                        intersection = False
                
                if len(Ystar) != 0:
                    # Find link
                    Xstar = tangent_polyline[left_tp_idx:] 
                    Ustar, Vstar = self.find_link(Xstar, Ystar, direction)
                    if not Ustar:
                        raise Exception("Cannot find link [u*, v*]")
                    
                    Ustar_idx = tangent_polyline.index(Ustar)
                    shortest_path += tangent_polyline[:Ustar_idx+1]
                    
                    start_index = dual_polyline.index(Vstar)
                    checking_pt_index = Ustar_idx
                    curr_polyline = dual_polyline
                    direction = not direction
                    break
                
                # No intersection
                tangent_polyline = tangent_polyline[:left_tp_idx+1]
                tangent_polyline.append(added_pt)
                if added_pt == self.polyline_P[-1]:
                    print("Reach goal")
                    print(tangent_polyline)
                    shortest_path += tangent_polyline
                    write_points_to_file(tangent_polyline, LOG_FILE)
                    return shortest_path
            print(tangent_polyline)
            write_points_to_file(tangent_polyline, LOG_FILE) # For illustration only
            
class SimplePolygonFromSequenceOfBundle(SimplePolygon):
    '''
    Class for representing a simple polygon constructed from a sequence of 
    bundles of line segments
    '''
    def __init__(self, sequence: SequenceOfBundles):
        polyline_P = [sequence.skeleton[0]]
        polyline_Q = [sequence.skeleton[0]]
        for i in range(1, len(sequence.skeleton)-1):
            # WARNING: Not degenerate bundles
            if sequence.outer_endpoints[i] == []:
                polyline_P.append(sequence.skeleton[i])
                polyline_Q.append(sequence.skeleton[i])
                continue
            if is_left(sequence.skeleton[i-1], 
                       sequence.skeleton[i],
                       sequence.outer_endpoints[i][0]
            ): 
                polyline_Q.append(sequence.skeleton[i])
                for outer_pt in sequence.outer_endpoints[i]:
                    polyline_P.append(outer_pt)
            else:
                polyline_P.append(sequence.skeleton[i])
                for outer_pt in sequence.outer_endpoints[i]:
                    polyline_Q.append(outer_pt)
        polyline_P.append(sequence.skeleton[-1])
        polyline_Q.append(sequence.skeleton[-1])
        super().__init__(polyline_P, polyline_Q)
