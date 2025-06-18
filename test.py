from utils import Point
from shortest_path import SimplePolygon
from main import visualize_shortest_path, visualize_simple_polygon, draw_convex_hull
from matplotlib import pyplot as plt

polyline_P = [
    Point(-4.04, 2.02),         # A
    Point(-0.423936727402, 0.8600951762659),  # B
    Point(1.2434497051775, 1.8220488873694),  # C
    Point(-0.16, 3.04),         # D
    Point(0.64, 6.6),           # E
    Point(5.14, 4.18),          # G
    Point(5.7966972710677, 0.795964928859),   # H
    Point(7.6244093221645, 1.9182442584798),  # I
    Point(8.939079394006, 1.5013976503349),   # J
]

# polyline_Q from J to A (in order)
polyline_Q = [
    Point(8.939079394006, 1.5013976503349),   # J
    Point(8.7146235280818, 0.6356393103417),  # K
    Point(5.4119157866263, -1.192072740755),  # L
    Point(4.257571333302, 2.8801979695834),   # M
    Point(3.1352920036812, 3.7459563095766),  # N
    Point(3.7445293540468, 0.7638998051555),  # O
    Point(2.269533663688, 2.3671559903281),   # P
    Point(2.4619244059087, 0.8600951762659),  # Q
    Point(1.4037753236948, -1.8013100911206), # R
    Point(-3.2456676133058, -1.6409844726034),# S
    Point(-4.04, 2.02),                        # A
][::-1]

# Create the polygon
polygon = SimplePolygon(polyline_P, polyline_Q)
shortest_path = polygon.find_shortest_path()

visualize_simple_polygon(plt, polygon)
draw_convex_hull(plt, polygon.convex_hulls)
visualize_shortest_path(plt, shortest_path)

plt.show()