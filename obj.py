import math
import logging
import matplotlib.pyplot as plt
from collections import deque
from utils import calculate_distance, is_larger_angle, \
                    is_equal_angle, is_smaller_angle, \
                    is_left, is_left_on, calculate_angle, \
                    Point, LineSegment
    
class SequenceOfBundles:
    """
    Class for representing a sequence of bundles
    
    Attributes:
        singleton (list[Point]): the singleton (a sequence of vertices)
        radius (float): the max length of line segment in bundles 
        outer_endpoints (list[list[Point]]): list of list of outer endpoinst of line segments
    """
    def __init__(self, singleton: list[Point], radius: float) -> None:
        self.singleton = singleton
        self.radius = radius
        self.outer_endpoints: list[list[Point]] = [[] for _ in range(len(self.singleton))]
        
    def add_line_segment(self, vertex: Point, outer_endpoint: Point) -> None:
        if not vertex in self.singleton:
            logging.error(f"Vertex {vertex} not in the singleton {self.singleton}")
            return        
        if vertex in [self.singleton[0], self.singleton[-1]]:
            logging.error(f"Cannot add line segments [{vertex}, {outer_endpoint}] into the first and last bundles.")
            return
        if calculate_distance(vertex, outer_endpoint) > self.radius:
            logging.error(f"The length of the added segment [{vertex}, {outer_endpoint}] is greater than the max radius.")
            return
        
        index = self.singleton.index(vertex)
        prev_vertex = self.singleton[index-1]
        next_vertex = self.singleton[index+1]
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
            
class SimplePolygon:
    def __init__(self, polyline_P: list[Point], polyline_Q: list[Point]):
        if (polyline_P[0] != polyline_Q[0] or 
            polyline_P[-1] != polyline_Q[-1]
        ):
            raise ValueError("Start and end point of two polylines must be the same.")
        self.polyline_P = polyline_P
        self.polyline_Q = polyline_Q
        # Future work: check intersection
        
    @staticmethod 
    def from_sequence_of_bundles(sequence: SequenceOfBundles):
        polyline_P = [sequence.singleton[0]]
        polyline_Q = [sequence.singleton[0]]
        for i, pt in list(enumerate(sequence.singleton))[1:-1]:
            # WARNING: Not degenerate bundles
            if is_left(sequence.singleton[i-1], 
                       sequence.singleton[i],
                       sequence.outer_endpoints[i][0]
            ): 
                polyline_P.append(sequence.singleton[i])
                for outer_pt in sequence.outer_endpoints[i]:
                    polyline_P.append(outer_pt)
            else:
                polyline_P.append(sequence.singleton[i])
                for outer_pt in sequence.outer_endpoints[i]:
                    polyline_Q.append(outer_pt)
        polyline_P.append(sequence.singleton[-1])
        polyline_Q.append(sequence.singleton[-1])
        return SimplePolygon(polyline_P, polyline_Q)
    
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
            if not is_left_on(Ustar, Vstar, pt,not direction):
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
        
    def find_shortest_path(self):
        shortest_path = []
        start_index = 0
        direction = 1 # Work on
        curr_polyline = self.polyline_P
        while True:
            dual_polyline = self.polyline_Q  \
                             if curr_polyline == self.polyline_P \
                             else self.polyline_P
            tangent_polyline = []                 
            # Assign first three point of the polyline
            if curr_polyline[start_index] == self.polyline_P[-1]:
                shortest_path += curr_polyline[start_index]
                return shortest_path
            elif curr_polyline[start_index+1] == self.polyline_P[-1]:
                shortest_path += [curr_polyline[start_index], curr_polyline[start_index+1]]
                return shortest_path
            pt0 = curr_polyline[start_index]
            pt1 = curr_polyline[start_index+1]
            pt2 = curr_polyline[start_index+2]
            # Initialize the convex hull
            if is_left(pt0, pt1, pt2, direction):
                tangent_polyline.append(pt0)
                tangent_polyline.append(pt1)
            else: 
                tangent_polyline.append(pt1)
                tangent_polyline.append(pt0)
            # tangent_polyline.append(pt2)
            # tangent_polyline.insert(0, pt2)
            
            # Check intersection before adding
            for i in range(start_index+2, len(curr_polyline)):
                added_pt = curr_polyline[i]

                left_tp_idx = len(tangent_polyline) - 1
                while not is_left(tangent_polyline[left_tp_idx-1], tangent_polyline[left_tp_idx], added_pt, direction) and left_tp_idx > 0:
                    left_tp_idx = left_tp_idx - 1
                right_tp_idx = 0
                while not is_left(added_pt, tangent_polyline[right_tp_idx], tangent_polyline[right_tp_idx+1], direction) and right_tp_idx < len(tangent_polyline)-1:
                    right_tp_idx = right_tp_idx + 1
                
                # # Check intersection
                Ystar = []
                for pt in dual_polyline:
                    if self.is_inside_new_hull(tangent_polyline[left_tp_idx], tangent_polyline[right_tp_idx], added_pt, pt, direction):
                        Ystar.append(pt)
                
                if len(Ystar) == 0:
                    # No intersection
                    # slicing the tangent_polyline
                    tangent_polyline = tangent_polyline[right_tp_idx:left_tp_idx+1]
                    tangent_polyline.append(added_pt)
                    tangent_polyline.insert(0, added_pt)
                    if added_pt == self.polyline_P[-1]:
                        print("Reach goal")
                        start_tp_idx = tangent_polyline.index(curr_polyline[start_index])
                        shortest_path += tangent_polyline[start_tp_idx:]
                        return shortest_path
                else:
                    # Find link
                    Xstar = tangent_polyline[0:right_tp_idx+1] + tangent_polyline[left_tp_idx:-1]
                    Ustar, Vstar = self.find_link(Xstar, Ystar, direction)
                    if not Ustar:
                        raise Exception("Cannot find link [u*, v*]")
                    
                    start_tp_idx = tangent_polyline.index(curr_polyline[start_index])
                    Ustar_idx = tangent_polyline.index(Ustar)
                    print(self.get_tangent_line(tangent_polyline, start_tp_idx, Ustar_idx))
                    shortest_path += self.get_tangent_line(tangent_polyline, start_tp_idx, Ustar_idx)

                    start_index = dual_polyline.index(Vstar)
                    curr_polyline = dual_polyline
                    direction = not direction
                    break
        
class SequenceVisualizer:
    @staticmethod
    def plot_sequence(sequence: SequenceOfBundles) -> None:
        """
        Visualize the sequence of bundles of line segments.
        
        Args:
            sequence (SequenceOfBundles): The sequence of bundles to visualize.
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # Plot the singleton (sequence of vertices)
        vertices = sequence.singleton
        x_vertices = [v.x for v in vertices]
        y_vertices = [v.y for v in vertices]
        ax.plot(x_vertices, y_vertices, 'ro-', label="Singleton (Vertices)")
        
        # Plot the line segments in each bundle
        for i, vertex in enumerate(vertices):
            # Skip the first and last vertices (no line segments allowed)
            if i == 0 or i == len(vertices) - 1:
                continue
            
            # Draw the line segments for this vertex
            outer_endpoints = sequence.outer_endpoints[i]
            for endpoint in outer_endpoints:
                ax.plot(
                    [vertex.x, endpoint.x],
                    [vertex.y, endpoint.y],
                    'b--',  # Dashed blue line
                    label="Line Segment" if i == 1 and endpoint == outer_endpoints[0] else None  # Label once
                )
                ax.scatter(endpoint.x, endpoint.y, color="green", label="Outer Endpoint" if i == 1 and endpoint == outer_endpoints[0] else None)

        # Add labels and a legend
        ax.set_title("Sequence of Bundles of Line Segments")
        ax.set_xlabel("X-coordinate")
        ax.set_ylabel("Y-coordinate")
        ax.legend()
        ax.grid(True)
        
        # Show the plot
        plt.show()
        
# Example Usage
if __name__ == "__main__":
    # Create example vertices
    vertices = [
        Point(0, 0),
        Point(2, 2),
        Point(4, 1),
        Point(6, 3),
        Point(8, 0),
    ]
    
    # Initialize sequence
    radius = 3
    sequence = SequenceOfBundles(vertices, radius)
    
    # Add line segments to the second vertex
    sequence.add_line_segment(vertices[1], Point(2, 0))
    sequence.add_line_segment(vertices[1], Point(3, 1))
    
    # Add line segments to the third vertex
    sequence.add_line_segment(vertices[2], Point(5, 3))
    sequence.add_line_segment(vertices[2], Point(3, 4))
    
    # Add line segments to the fourth vertex
    sequence.add_line_segment(vertices[3], Point(7, 1))
    sequence.add_line_segment(vertices[3], Point(5, 0.5))
    
    # Visualize the sequence
    SequenceVisualizer.plot_sequence(sequence)   
        
    
    