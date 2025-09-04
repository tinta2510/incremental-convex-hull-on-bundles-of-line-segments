import os
import subprocess
import sys
from shortest_path import SimplePolygon, SequenceOfBundles, Point, SimplePolygonFromSequenceOfBundle
import matplotlib.pyplot as plt

def run_cpp_code():
    """Compile and run the C++ code to generate the shortest path log file."""
    print("Compiling C++ code...")
    
    # Navigate to the cpp directory
    os.chdir("cpp")
    
    # Compile the C++ code
    compile_process = subprocess.run(
        ["g++", "-std=c++17", "-o", "main", "main.cpp", "shortest_path.cpp", "utils.cpp"],
        capture_output=True,
        text=True
    )
    
    if compile_process.returncode != 0:
        print("Compilation failed:")
        print(compile_process.stderr)
        os.chdir("..")  # Go back to the original directory
        return False
    
    print("Compilation successful. Running the C++ program...")
    
    # Run the compiled program
    run_process = subprocess.run(
        ["./main.exe" if os.name == 'nt' else "./main"],
        capture_output=True,
        text=True
    )
    
    if run_process.returncode != 0:
        print("C++ program execution failed:")
        print(run_process.stderr)
        os.chdir("..")  # Go back to the original directory
        return False
    
    print("C++ program execution successful:")
    print(run_process.stdout)
    
    # Go back to the original directory
    os.chdir("..")
    return True

def visalize_sequence(plt, sequence: SequenceOfBundles):
    """Visualize the sequence of bundles."""
    # Plot skeleton points
    skeleton_x = [pt.x for pt in sequence.skeleton]
    skeleton_y = [pt.y for pt in sequence.skeleton]
    plt.plot(skeleton_x, skeleton_y, 'bo-', label='Skeleton', linewidth=2.5)

    # Plot outer endpoints and line segments
    for i, outer_points in enumerate(sequence.outer_endpoints):
        for outer_pt in outer_points:
            plt.plot([sequence.skeleton[i].x, outer_pt.x],
                    [sequence.skeleton[i].y, outer_pt.y],
                    'r--', alpha=0.6)

def visualize_shortest_path(plt, shortest_path: list[Point]):
    """Visualize the shortest path on the sequence of bundles."""
    shortest_path_x = [pt.x for pt in shortest_path]
    shortest_path_y = [pt.y for pt in shortest_path]
    plt.plot(shortest_path_x, shortest_path_y, 'g-', label='Shortest Path', linewidth=3)

def read_shortest_path_from_file(filename):
    """
    Read shortest path coordinates from a log file.
    
    Args:
        filename (str): Path to the log file.
        
    Returns:
        list[Point]: List of Point objects representing the shortest path.
    """
    shortest_path = []
    
    with open(filename, 'r') as file:
        for line in file:
            # Skip comment lines
            if line.startswith('#'):
                continue
                
            # Parse coordinate lines
            line = line.strip()
            if line:
                try:
                    x, y = map(float, line.split())
                    shortest_path.append(Point(x, y))
                except ValueError:
                    print(f"Warning: Could not parse line: {line}")
    
    return shortest_path

def visualize_results():
    """Visualize the results using matplotlib."""
    print("Loading sequence data...")
    sequence = SequenceOfBundles.load_sequence_from_file("input/input_3.txt", preprocess=False)
    
    print("Reading the shortest path from the log file...")
    shortest_path = read_shortest_path_from_file("cpp/shortest_path_log.txt")
    
    if not shortest_path:
        print("No shortest path data found in the log file.")
        return False
    
    print(f"Found {len(shortest_path)} points in the shortest path.")
    
    print("Creating visualization...")
    plt.figure(figsize=(10, 8))
    visalize_sequence(plt, sequence)
    visualize_shortest_path(plt, shortest_path)
    
    plt.xticks([])  # Remove x-axis numbers
    plt.yticks([])  # Remove y-axis numbers
    plt.grid(False)
    plt.axis('equal')
    plt.suptitle("Sequence Visualization with Shortest Path (C++ Implementation)")
    plt.legend()
    
    print("Displaying visualization...")
    plt.show()
    return True

if __name__ == "__main__":
    print("=== Running C++ Implementation and Visualizing Results ===")
    

    cpp_success = run_cpp_code()
    if not cpp_success:
        print("Failed to run C++ code. Trying to visualize existing results anyway...")
    
    # Visualize the results
    vis_success = visualize_results()
    
    if not vis_success:
        print("Visualization failed.")
        sys.exit(1)
    
    print("Process completed successfully!")