#ifndef MAP_HPP
#define MAP_HPP

#include <string>
#include <vector>
#include <tuple>
#include <optional>
#include <utility>
#include <memory>

namespace plt {
    class Axes;
    class Figure;
}

// Forward declaration for Boost types
namespace boost {
    namespace geometry {
        namespace model {
            namespace d2 {
                template <typename T> class point_xy;
            }
            template <typename Point> class polygon;
        }
    }
}

// Define Point and Segment structures
struct Point {
    double x;
    double y;
};

using Segment = std::pair<Point, Point>;

// Function type for dynamic library segment intersection
using SegmentIntersectFunc = bool (*)(
    double*, double*,  // Output intersection point
    double, double,    // First segment start
    double, double,    // First segment end
    double, double,    // Second segment start
    double, double     // Second segment end
);

class Intersect {
public:
    static std::optional<Point> segment_intersect(const Segment& segment1, const Segment& segment2);
};

class Map {
public:
    Map(const std::string& json_file,
        double sensor_range,
        double safety_distance = 0.0,
        bool use_regions = false,
        bool compiled_intersect = true);
    
    ~Map();

    // Map query functions
    std::tuple<double, double, double, double> bounds() const;
    std::pair<Point, double> check_collision(const Segment& segment, bool compute_distance = false) const;
    bool contains(const Point& point) const;
    bool crosses(const Segment& segment) const;
    const std::optional<std::vector<std::vector<int>>>& getGridMap() const;

    // Visualization functions
    void plot(plt::Axes& axes) const;
    void show(const std::string& title,
              int figure_number = 1,
              bool block = true,
              std::pair<float, float> figure_size = {7.0, 7.0},
              bool save_figure = false,
              const std::string& save_dir = "maps") const;
    void showRegions(const std::string& title,
                    int figure_number = 1,
                    bool block = true,
                    std::pair<float, float> figure_size = {7.0, 7.0},
                    bool save_figure = false,
                    const std::string& save_dir = "maps") const;

private:
    // Polygon representation
    std::vector<Point> boundary_;
    std::vector<std::vector<Point>> holes_;
    std::vector<Segment> map_segments_;
    std::optional<std::vector<std::vector<int>>> grid_map_;
    
    // Boost geometry types
    using BoostPoint = boost::geometry::model::d2::point_xy<double>;
    using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
    
    BoostPolygon map_polygon_;
    BoostPolygon safe_map_polygon_;

    // Performance optimizations
    double sensor_range_;
    void* intersect_handle_ = nullptr;
    SegmentIntersectFunc segment_intersect_ = nullptr;
    mutable double XC_, YR_;
    std::vector<std::vector<std::vector<Segment>>> region_segments_;

    // Helper methods
    void initIntersect();
    std::vector<std::vector<std::vector<Segment>>> initRegions() const;
    std::pair<int, int> xyToRC(const Point& xy) const;
    BoostPolygon createBoostPolygon(const std::vector<Point>& boundary, 
                                     const std::vector<std::vector<Point>>& holes) const;
    BoostPolygon createBufferedPolygon(const BoostPolygon& poly, double distance) const;
};

#endif // MAP_HPP