from utils import Point
from shortest_path import SimplePolygon, SequenceOfBundles
import matplotlib.pyplot as plt

def visualize_sequence_of_bundles(sequence: SequenceOfBundles):
    """Visualize the sequence of bundles."""
    fig, ax = plt.subplots()

    # Plot singleton points
    singleton_x = [pt.x for pt in sequence.singleton]
    singleton_y = [pt.y for pt in sequence.singleton]
    ax.plot(singleton_x, singleton_y, 'bo-', label='Singleton')

    # Plot outer endpoints and line segments
    for i, outer_points in enumerate(sequence.outer_endpoints):
        for outer_pt in outer_points:
            ax.plot([sequence.singleton[i].x, outer_pt.x],
                    [sequence.singleton[i].y, outer_pt.y],
                    'r-', alpha=0.6)

    ax.set_title("Sequence of Bundles")
    ax.legend()
    plt.show()


def visualize_shortest_path(sequence: SequenceOfBundles, shortest_path: list[Point]):
    """Visualize the shortest path on the sequence of bundles."""
    fig, ax = plt.subplots()

    # Plot singleton points
    singleton_x = [pt.x for pt in sequence.singleton]
    singleton_y = [pt.y for pt in sequence.singleton]
    ax.plot(singleton_x, singleton_y, 'bo-', label='Singleton')

    # Plot outer endpoints and line segments
    for i, outer_points in enumerate(sequence.outer_endpoints):
        for outer_pt in outer_points:
            ax.plot([sequence.singleton[i].x, outer_pt.x],
                    [sequence.singleton[i].y, outer_pt.y],
                    'r-', alpha=0.6)

    # Plot shortest path
    shortest_path_x = [pt.x for pt in shortest_path]
    shortest_path_y = [pt.y for pt in shortest_path]
    ax.plot(shortest_path_x, shortest_path_y, 'g-', label='Shortest Path')

    ax.set_title("Shortest Path Visualization")
    ax.legend()
    plt.show()

if __name__=="__main__":
    
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
    
    polygon = SimplePolygon.from_sequence_of_bundles(sequence)
    shortest_path = polygon.find_shortest_path()
    print(shortest_path)
    visualize_shortest_path(sequence, shortest_path)