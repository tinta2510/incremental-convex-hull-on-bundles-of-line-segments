from shortest_path import SimplePolygon, SequenceOfBundles, Point
import matplotlib.pyplot as plt

def visalize_sequence(plt, sequence: SequenceOfBundles):
    """Visualize the sequence of bundles."""
    # Plot singleton points
    singleton_x = [pt.x for pt in sequence.singleton]
    singleton_y = [pt.y for pt in sequence.singleton]
    plt.plot(singleton_x, singleton_y, 'bo--', label='Skeleton', linewidth=2.5)

    # Plot outer endpoints and line segments
    for i, outer_points in enumerate(sequence.outer_endpoints):
        for outer_pt in outer_points:
            plt.plot([sequence.singleton[i].x, outer_pt.x],
                    [sequence.singleton[i].y, outer_pt.y],
                    'r--', alpha=0.6)

def visualize_shortest_path(plt, shortest_path: list[Point]):
    """Visualize the shortest path on the sequence of bundles."""
    shortest_path_x = [pt.x for pt in shortest_path]
    shortest_path_y = [pt.y for pt in shortest_path]
    plt.plot(shortest_path_x, shortest_path_y, 'g-', label='Shortest Path', linewidth=2)
    
def visualize_simple_polygon(plt, polygon: SimplePolygon):
    # Plot polyline P
    polyline_P_x = [pt.x for pt in polygon.polyline_P]
    polyline_P_y = [pt.y for pt in polygon.polyline_P]
    plt.plot(polyline_P_x, polyline_P_y, 'o-', label='_nolegend_', linewidth=1, color='gray')
    
    # Plot polyline Q
    polyline_Q_x = [pt.x for pt in polygon.polyline_Q]
    polyline_Q_y = [pt.y for pt in polygon.polyline_Q]
    plt.plot(polyline_Q_x, polyline_Q_y, 'o-', label='_nolegend_', linewidth=1, color='gray')
    
def read_convex_hull_from_file(filename):
    """
    Read points and polygons from a file.
    :param filename: The name of the file containing points.
    :return: A list of polygons, where each polygon is a list of (x, y) tuples.
    """
    polygons = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        current_polygon = []
        for line in lines:
            line = line.strip()
            if not line:  # Empty line indicates the end of a polygon
                if current_polygon:
                    polygons.append(current_polygon)
                    current_polygon = []
            else:
                coords = line.split()
                if len(coords) == 2:
                    x, y = map(float, coords)
                    current_polygon.append((x, y))
        if current_polygon:  # Add the last polygon if not already added
            polygons.append(current_polygon)
    return polygons

def draw_convex_hull(plt, polygons):
    """
    Draw polygons using matplotlib.
    :param polygons: A list of polygons, where each polygon is a list of (x, y) tuples.
    """
    for _, polygon in enumerate(polygons):
        x_coords, y_coords = zip(*polygon)  # Unpack the points
        plt.fill(x_coords, y_coords, color='orange', label="Convex Hull")
    
    
if __name__=="__main__":
    sequence = SequenceOfBundles.load_sequence_from_file("input/input_1.txt", preprocess=False)
    polygon = SimplePolygon.from_sequence_of_bundles(sequence)
    shortest_path = polygon.find_shortest_path(False)
    
    visalize_sequence(plt, sequence)
    # visualize_shortest_path(plt, shortest_path)
    visualize_simple_polygon(plt, polygon)
    
    # # Visualize the convex hull
    # filename = "convex_hull.log"
    # ## Read points and polygons from the file
    # polygons = read_convex_hull_from_file(filename)
    # ## Draw the polygons
    # draw_convex_hull(plt, polygons)
    
    plt.xticks([])  # Remove x-axis numbers
    plt.yticks([])  # Remove y-axis numbers
    plt.grid(False)
    plt.axis('equal')
    plt.suptitle("Sequence Visualization")
    plt.legend()
    plt.show() 