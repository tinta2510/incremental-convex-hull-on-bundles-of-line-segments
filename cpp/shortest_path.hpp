#ifndef SHORT_PATH_HPP
#define SHORT_PATH_HPP

#include <utils.hpp>

#include <vector>
#include <optional>
#include <utility>

struct SequenceOfBundles {
    std::vector<Point> skeleton;
    float radius;
    std::vector<std::vector<Point>> outer_endpoints;

    SequenceOfBundles(const std::vector<Point>&, float);
    SequenceOfBundles(std::vector<Point>&&, float);

    void addOuterEndpoint(const Point&, const Point&);
    void preprocess();
};

struct SimplePolygon {
    std::vector<Point> polyline_P;
    std::vector<Point> polyline_Q;
    std::vector<std::vector<Point>> convex_hulls;

    SimplePolygon(const std::vector<Point>&, const std::vector<Point>&);
    SimplePolygon(std::vector<Point>&&, std::vector<Point>&&);

    std::vector<Point> findShortestPath(bool);
protected: 
    void initialize();

    bool isInsideNewHull(const Point& left_tp, const Point& right_tp, 
                        const Point& added_pt, const Point& checking_pt, bool direction) const;
    
    bool verifyLink(const std::vector<Point>& Xstar, const std::vector<Point>& Ystar,
                   const Point& Ustar, const Point& Vstar, bool direction) const;
    
    std::pair<std::optional<Point>, std::optional<Point>> findLink(
        const std::vector<Point>& Xstar, const std::vector<Point>& Ystar, bool direction) const;
    
    std::vector<Point> getTangentLine(const std::vector<Point>& tangent_polyline,
                                     size_t start_tp_idx, size_t end_tp_idx) const;
    
    std::pair<size_t, size_t> findTangentPoints(const std::vector<Point>& tangent_polyline,
                                               const Point& added_pt, bool direction) const;
    
    size_t findLeftTangentPoint(const std::vector<Point>& tangent_polyline,
                               const Point& added_pt, bool direction) const;
}

#endif // SHORT_PATH_HPP