import logging
from itertools import combinations
from functools import reduce
from utils import (
    calculate_distance, is_larger_angle, is_equal_angle, is_left, is_left_on, 
    calculate_angle, Point, do_intersect, ConvexHull
)
from intervaltree import IntervalTree
                   
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
    
    def _partition_into_convex_subpolylines(self) -> list[list[int]]:
        convex_ropes = []
        curr_rope = [0, 1, 2]
        left_flag = is_left_on(self.skeleton[0], self.skeleton[1], self.skeleton[2])
        for i in range(3, len(self.skeleton)):
            # Check if the current point is on the left or right of the previous segment
            direction = is_left_on(self.skeleton[i-1], self.skeleton[i], self.skeleton[i-2])
            if direction == left_flag:
                curr_rope.append(i)
            else:
                convex_ropes.append(curr_rope)
                curr_rope = [i-2, i-1, i]
                left_flag = direction
        convex_ropes.append(curr_rope)  # Add the last rope
        return convex_ropes
    
    def find_shortest_path(self) -> list[Point]:
        """
        Find the shortest path in the sequence of bundles by concatenating local convex hulls.
        
        :return: List of Point objects representing the shortest path.
        """
        convex_ropes = self._partition_into_convex_subpolylines()
        shortest_path = [self.skeleton[0]]  # Start with the first skeleton point
        local_CHs = []
        for rope in convex_ropes:
            endpoints = ([self.skeleton[rope[0]]] 
                        + reduce(lambda acc, ele: acc + ele, 
                                 [self.outer_endpoints[i] for i in rope[1:-1]], []) 
                        + [self.skeleton[rope[-1]]] )
            convex_hull = ConvexHull.incremental_convex_hull(endpoints)
            clockwise = not is_left_on(self.skeleton[rope[0]], self.skeleton[rope[1]], self.skeleton[rope[2]])
            convex_rope = ConvexHull.extract_convex_rope_from_hull(
                convex_hull, self.skeleton[rope[0]], self.skeleton[rope[-1]], 
                clockwise=clockwise
            )
            local_CHs.append(convex_rope)
            
        # for i in range(len(local_CHs)-2):
        #     link1 = ConvexHull.find_tangent(local_CHs[i], local_CHs[i+1])
        #     link2 = ConvexHull.find_tangent(local_CHs[i+1], local_CHs[i+2])
        #     if do_intersect(link1[0], link1[1], link2[0], link2[1]):
        #         #!!! Check case two segments have one same endpoint
        #         if link1[1] == link2[0]:
        #             raise NotImplementedError("The case where two segments have one same endpoint is not implemented yet.")
        #         else:
        #             link = ConvexHull.find_tangent(local_CHs[i], local_CHs[i+2], external=True)
        #             shortest_path.append() #!!!

        #     else:
        while len(local_CHs) > 2:
            link1 = ConvexHull.find_tangent(local_CHs[0], local_CHs[1])
            link2 = ConvexHull.find_tangent(local_CHs[1], local_CHs[2])
            if do_intersect(link1[0], link1[1], link2[0], link2[1]):
                # Check if the two segments have one same endpoint
                if link1[1] == link2[0]:
                    #!!! Handle later
                    shortest_path.extend(ConvexHull.extract_convex_rope_from_hull(
                        local_CHs[0], shortest_path[-1], link1[0], 
                        # clockwise=not is_left_on(local_CHs[0][0], local_CHs[0][1], local_CHs[0][2])
                    )[1:])
                    local_CHs.pop(0)
                    shortest_path.append(link1[1])
                else:
                    # Find the external tangent between the first and third convex ropes
                    link = ConvexHull.find_tangent(local_CHs[0], local_CHs[2], external=True)
                    new_hull =  ConvexHull.extract_convex_rope_from_hull(
                        local_CHs[0], local_CHs[0][0], link[0], 
                        # clockwise=not is_left_on(local_CHs[0][0], local_CHs[0][1], local_CHs[0][2])
                    ) + ConvexHull.extract_convex_rope_from_hull(
                        local_CHs[2], link[1], local_CHs[2][-1], 
                        # clockwise=not is_left_on(local_CHs[2][0], local_CHs[2][1], local_CHs[2][2])
                    )
                    [local_CHs.pop(0) for _ in range(3)]
                    local_CHs.insert(0, new_hull)
            else:
                shortest_path.extend(ConvexHull.extract_convex_rope_from_hull(
                    local_CHs[0], shortest_path[-1], link1[0], 
                    # clockwise=not is_left_on(local_CHs[0][0], local_CHs[0][1], local_CHs[0][2])
                )[1:])
                local_CHs.pop(0)
                shortest_path.append(link1[1])
                
        if len(local_CHs) == 2:
            link1 = ConvexHull.find_tangent(local_CHs[0], local_CHs[1])
            shortest_path.extend(ConvexHull.extract_convex_rope_from_hull(
                local_CHs[0], shortest_path[-1], link1[0], 
                # clockwise=not is_left_on(local_CHs[0][0], local_CHs[0][1], local_CHs[0][2])
            )[1:])
            shortest_path.append(link1[1])
            shortest_path.extend(ConvexHull.extract_convex_rope_from_hull(
                local_CHs[1], shortest_path[-1], local_CHs[1][-1], 
                # clockwise=not is_left_on(local_CHs[1][0], local_CHs[1][1], local_CHs[1][2])
            )[1:])
        else:           
            shortest_path.extend(ConvexHull.extract_convex_rope_from_hull(
                                    local_CHs[0], shortest_path[-1], local_CHs[0][-1], 
                                    # clockwise=not is_left_on(local_CHs[0][0], local_CHs[0][1], local_CHs[0][2])
                                )[1:])
        return shortest_path
        
        