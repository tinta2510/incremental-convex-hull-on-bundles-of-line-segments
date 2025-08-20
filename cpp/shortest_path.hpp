#ifndef SHORT_PATH_HPP
#define SHORT_PATH_HPP

#include <vector>
#include <utils.hpp>

struct SequenceOfBundles {
    std::vector<Point> skeleton;
    float radius;
    std::vector<std::vector<Point>> outer_endpoints;

    SequenceOfBundles(const std::vector<Point>&, float);
    SequenceOfBundles(std::vector<Point>&&, float);

    void addOuterEndpoint(const Point&, const Point&);
};

#endif // SHORT_PATH_HPP