#include "map.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <stdexcept>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/strategies/buffer/distance_symmetric.hpp>
#include <algorithm>
#include <filesystem>

using json = nlohmann::json;
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> BoostPoint;
typedef bg::model::polygon<BoostPoint> BoostPolygon;
typedef bg::model::linestring<BoostPoint> BoostLine;

Map::Map(const std::string& json_file,
         double sensor_range,
         double safety_distance,
         bool use_regions,
         bool compiled_intersect)
    : sensor_range_(sensor_range) {

    std::ifstream file(json_file);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open map file: " + json_file);
    }

    json data;
    file >> data;

    // Load boundary and holes
    auto boundary_data = data["metric"]["boundary"];
    auto holes_data = data["metric"]["holes"];

    // Create boundary points
    for (const auto& point : boundary_data) {
        boundary_.emplace_back(Point{point[0], point[1]});
    }

    // Create holes
    for (const auto& hole : holes_data) {
        std::vector<Point> hole_polygon;
        for (const auto& point : hole) {
            hole_polygon.emplace_back(Point{point[0], point[1]});
        }
        holes_.emplace_back(hole_polygon);
    }

    // Create polygon representation
    map_polygon_ = createBoostPolygon(boundary_, holes_);
    
    // Create safe map polygon with buffer
    safe_map_polygon_ = createBufferedPolygon(map_polygon_, -safety_distance);

    // Create segment map
    for (size_t i = 0; i < boundary_.size(); ++i) {
        Point p1 = boundary_[i];
        Point p2 = boundary_[(i + 1) % boundary_.size()];
        map_segments_.emplace_back(Segment{p1, p2});
    }

    for (const auto& hole : holes_) {
        for (size_t i = 0; i < hole.size(); ++i) {
            Point p1 = hole[i];
            Point p2 = hole[(i + 1) % hole.size()];
            map_segments_.emplace_back(Segment{p1, p2});
        }
    }

    // Try to create grid map if available
    try {
        auto map_size = data["grid"]["size"];
        auto obstacles = data["grid"]["obstacles"];
        int rows = map_size[0];
        int cols = map_size[1];

        std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));

        for (const auto& obstacle : obstacles) {
            int r = obstacle[0];
            int c = obstacle[1];
            if (r >= 0 && r < rows && c >= 0 && c < cols) {
                grid[r][c] = 1;
            }
        }
        
        grid_map_ = grid;
    } catch (...) {
        // Grid map not available
        grid_map_ = std::nullopt;
    }

    // Initialize performance optimizations
    if (compiled_intersect) {
        initIntersect();
    } else {
        intersect_handle_ = nullptr;
        segment_intersect_ = nullptr;
    }

    if (use_regions) {
        region_segments_ = initRegions();
    }
}

std::tuple<double, double, double, double> Map::bounds() const {
    double x_min = std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_max = std::numeric_limits<double>::lowest();

    // Use Boost Geometry to get the bounding box
    bg::model::box<BoostPoint> box;
    bg::envelope(map_polygon_, box);
    
    return std::make_tuple(box.min_corner().x(), box.min_corner().y(), 
                          box.max_corner().x(), box.max_corner().y());
}

std::pair<Point, double> Map::check_collision(const Segment& segment, bool compute_distance) const {
    std::vector<Point> intersections;
    double distance = std::numeric_limits<double>::quiet_NaN();
    size_t index = 0;

    try {
        const std::vector<Segment>* segments_to_check = &map_segments_;

        // If regions are used, find the appropriate region
        if (!region_segments_.empty()) {
            auto [r, c] = xyToRC(segment.first);
            
            // Check if region indices are valid
            if (r >= 0 && c >= 0 && r < static_cast<int>(region_segments_.size()) && 
                c < static_cast<int>(region_segments_[0].size())) {
                segments_to_check = &region_segments_[r][c];
            } else {
                throw std::out_of_range("Region indices out of bounds");
            }
        }

        // Check for intersections using compiled library if available
        if (segment_intersect_ != nullptr) {
            double xi = 0.0, yi = 0.0;
            
            for (const auto& map_segment : *segments_to_check) {
                bool found = segment_intersect_(
                    &xi, &yi,
                    segment.first.x, segment.first.y,
                    segment.second.x, segment.second.y,
                    map_segment.first.x, map_segment.first.y,
                    map_segment.second.x, map_segment.second.y
                );

                if (found) {
                    intersections.emplace_back(Point{xi, yi});
                }
            }
        } else {
            // Use the Intersect class if compiled library not available
            for (const auto& map_segment : *segments_to_check) {
                auto pt_opt = Intersect::segment_intersect(segment, map_segment);
                if (pt_opt.has_value()) {
                    intersections.push_back(pt_opt.value());
                }
            }
        }

        // Calculate distance if needed or multiple intersections found
        if ((compute_distance && !intersections.empty()) || intersections.size() > 1) {
            std::vector<double> distances;
            
            for (const auto& pt : intersections) {
                double dx = pt.x - segment.first.x;
                double dy = pt.y - segment.first.y;
                distances.push_back(std::sqrt(dx * dx + dy * dy));
            }

            auto min_it = std::min_element(distances.begin(), distances.end());
            index = std::distance(distances.begin(), min_it);
            distance = *min_it;
        }
    } catch (const std::out_of_range&) {
        // Handle out of range exceptions silently
        // Sensor rays may be outside the map even if robot center is within
    }

    // Return the closest intersection or empty point if none
    if (!intersections.empty()) {
        return {intersections[index], distance};
    } else {
        return {Point{}, distance};
    }
}

bool Map::contains(const Point& point) const {
    BoostPoint bp(point.x, point.y);
    return bg::within(bp, safe_map_polygon_);
}

bool Map::crosses(const Segment& segment) const {
    BoostLine line;
    line.push_back(BoostPoint(segment.first.x, segment.first.y));
    line.push_back(BoostPoint(segment.second.x, segment.second.y));

    return bg::crosses(line, safe_map_polygon_);
}

const std::optional<std::vector<std::vector<int>>>& Map::getGridMap() const {
    return grid_map_;
}

void Map::plot(plt::Axes& axes) const {
    auto [x_min, y_min, x_max, y_max] = bounds();

    // Draw grid
    std::vector<double> major_ticks;
    for (double t = std::floor(std::min(x_min, y_min)); 
         t <= std::ceil(std::max(x_max, y_max)) + 0.01; t += 1.0) {
        major_ticks.push_back(t);
    }
    
    std::vector<double> minor_ticks;
    for (double t = std::floor(std::min(x_min, y_min)); 
         t <= std::ceil(std::max(x_max, y_max)) + 0.01; t += 0.5) {
        minor_ticks.push_back(t);
    }
    
    axes.set_xticks(major_ticks);
    axes.set_xticks(minor_ticks, true);  // minor=true
    axes.set_yticks(major_ticks);
    axes.set_yticks(minor_ticks, true);  // minor=true
    
    axes.set_xlim(std::floor(x_min), std::ceil(x_max));
    axes.set_ylim(std::floor(y_min), std::ceil(y_max));
    axes.grid(true, "both", 0.33, "dashed", 1);
    axes.set_labels("x [m]", "y [m]");
    
    // Plot safe map polygon
    std::vector<double> safe_x, safe_y;
    for (const auto& point : bg::exterior_ring(safe_map_polygon_)) {
        safe_x.push_back(point.x());
        safe_y.push_back(point.y());
    }
    axes.plot(safe_x, safe_y, "gray", 1.0, 3.0, "round", 2);
    
    // Plot map polygon boundary
    std::vector<double> x, y;
    for (const auto& point : bg::exterior_ring(map_polygon_)) {
        x.push_back(point.x());
        y.push_back(point.y());
    }
    axes.plot(x, y, "black", 1.0, 3.0, "round", 3);
    
    // Plot holes
    for (int i = 0; i < bg::num_interior_rings(map_polygon_); ++i) {
        const auto& interior = bg::interior_rings(map_polygon_)[i];
        const auto& safe_interior = bg::interior_rings(safe_map_polygon_)[i];
        
        std::vector<double> safe_hole_x, safe_hole_y;
        for (const auto& point : safe_interior) {
            safe_hole_x.push_back(point.x());
            safe_hole_y.push_back(point.y());
        }
        axes.plot(safe_hole_x, safe_hole_y, "gray", 1.0, 3.0, "round", 2);
        
        std::vector<double> hole_x, hole_y;
        for (const auto& point : interior) {
            hole_x.push_back(point.x());
            hole_y.push_back(point.y());
        }
        axes.plot(hole_x, hole_y, "black", 1.0, 3.0, "round", 3);
    }
}

void Map::show(const std::string& title,
               int figure_number,
               bool block,
               std::pair<float, float> figure_size,
               bool save_figure,
               const std::string& save_dir) const
{
    plt::Figure figure(figure_number, figure_size);
    plt::Axes axes = figure.add_subplot(1, 1, 1);
    
    plot(axes);
    axes.set_title("Map (" + title + ")");
    figure.tight_layout();
    
    figure.show(block);
    figure.pause(0.0001);  // Wait a bit to ensure display
    
    if (save_figure) {
        std::filesystem::path save_path = 
            std::filesystem::path(__FILE__).parent_path().parent_path() / save_dir;
            
        if (!std::filesystem::exists(save_path)) {
            std::filesystem::create_directories(save_path);
        }
        
        std::string file_name = title;
        std::transform(file_name.begin(), file_name.end(), file_name.begin(), ::tolower);
        file_name += ".png";
        
        std::filesystem::path file_path = save_path / file_name;
        figure.savefig(file_path.string());
    }
}

void Map::showRegions(const std::string& title,
                     int figure_number,
                     bool block,
                     std::pair<float, float> figure_size,
                     bool save_figure,
                     const std::string& save_dir) const
{
    auto [x_min, y_min, x_max, y_max] = bounds();
    int rows = region_segments_.size();
    int cols = region_segments_[0].size();
    
    float label_size, map_line_width, marker_size;
    
    if (rows <= 5 && cols <= 5) {
        label_size = 8.0f;
        map_line_width = 2.0f;
        marker_size = 5.0f;
    } else {
        label_size = 5.0f;
        map_line_width = 1.75f;
        marker_size = 1.5f;
    }
    
    plt::Figure figure(figure_number, figure_size);
    std::vector<std::vector<plt::Axes>> axes_grid(rows, std::vector<plt::Axes>(cols));
    
    // Create subplot grid
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            axes_grid[r][c] = figure.add_subplot(rows, cols, r * cols + c + 1, 
                                                "sharex", "sharey", 
                                                (rows > 5 || cols > 5));
            
            plt::Axes& ax = axes_grid[r][c];
            
            // Configure axes
            ax.set_labels("x [m]", "y [m]", "small");
            ax.label_outer();  // Hide labels for inner plots
            
            std::vector<double> major_ticks;
            for (double t = x_min; t <= x_max + 0.01; t += 1.0) {
                major_ticks.push_back(t);
            }
            
            ax.set_xticks(major_ticks);
            ax.set_yticks(major_ticks);
            ax.set_xlim(x_min, x_max);
            ax.set_ylim(y_min, y_max);
            
            ax.tick_params("x", label_size, 90);
            ax.tick_params("y", label_size);
            ax.grid(true, "both", 0.33, "dashed", 1);
        }
    }
    
    figure.suptitle("Map regions (" + title + ")");
    figure.tight_layout();
    
    // Draw regions
    for (double y = y_max - 0.5; y > y_min; y -= 1.0) {
        for (double x = x_min + 0.5; x < x_max; x += 1.0) {
            auto [r, c] = xyToRC({x, y});
            
            if (r >= 0 && r < rows && c >= 0 && c < cols) {
                // Draw center point
                axes_grid[r][c].plot({x}, {y}, "bo", marker_size);
                
                // Draw sensor range circle
                std::vector<double> cx, cy;
                const int circle_points = 50;
                for (int i = 0; i <= circle_points; ++i) {
                    double angle = 2 * M_PI * i / circle_points;
                    double radius = sensor_range_ + 1.0 / std::sqrt(2.0);
                    cx.push_back(x + radius * std::cos(angle));
                    cy.push_back(y + radius * std::sin(angle));
                }
                axes_grid[r][c].plot(cx, cy, "green", 0.5, 1.0, "dashed", 3);
                
                // Draw visible segments
                for (const auto& s : region_segments_[r][c]) {
                    axes_grid[r][c].plot({s.first.x, s.second.x}, 
                                        {s.first.y, s.second.y}, 
                                        "black", 1.0, map_line_width, "solid", 2);
                }
            }
        }
    }
    
    figure.show(block);
    
    if (save_figure) {
        std::filesystem::path save_path = 
            std::filesystem::path(__FILE__).parent_path().parent_path() / save_dir;
            
        if (!std::filesystem::exists(save_path)) {
            std::filesystem::create_directories(save_path);
        }
        
        std::string file_name = title;
        std::transform(file_name.begin(), file_name.end(), file_name.begin(), ::tolower);
        file_name += "_regions.png";
        
        std::filesystem::path file_path = save_path / file_name;
        figure.savefig(file_path.string());
    }
}

void Map::initIntersect() {
    std::string lib_name;
    
#ifdef _WIN32
    lib_name = "libintersect.dll";
#elif __APPLE__
    lib_name = "libintersect.dylib";
#else
    lib_name = "libintersect.so";
#endif

    std::filesystem::path lib_path = std::filesystem::path(__FILE__).parent_path() / lib_name;

#ifdef _WIN32
    intersect_handle_ = LoadLibrary(lib_path.string().c_str());
    if (!intersect_handle_) {
        throw std::runtime_error("Could not load " + lib_name);
    }

    segment_intersect_ = (SegmentIntersectFunc)GetProcAddress((HMODULE)intersect_handle_, "segment_intersect");
    if (!segment_intersect_) {
        throw std::runtime_error("Cannot load symbol 'segment_intersect'");
    }
#else
    intersect_handle_ = dlopen(lib_path.c_str(), RTLD_LAZY);
    if (!intersect_handle_) {
        throw std::runtime_error("Could not load " + lib_name + ": " + dlerror());
    }

    segment_intersect_ = (SegmentIntersectFunc)dlsym(intersect_handle_, "segment_intersect");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        throw std::runtime_error("Cannot load symbol 'segment_intersect': " + std::string(dlsym_error));
    }
#endif
}

std::vector<std::vector<std::vector<Segment>>> Map::initRegions() const {
    auto [x_min, y_min, x_max, y_max] = bounds();

    int map_rows = static_cast<int>(std::ceil(y_max - y_min));
    int map_cols = static_cast<int>(std::ceil(x_max - x_min));

    // Precomputed constants for faster (x,y) to (row,col) conversion
    XC_ = std::floor(map_cols / 2.0);
    YR_ = map_rows - std::ceil(map_rows / 2.0);

    std::vector<std::vector<std::vector<Segment>>> region_segments(map_rows,
        std::vector<std::vector<Segment>>(map_cols));

    for (double y = y_max - 0.5; y > y_min; y -= 1.0) {
        for (double x = x_min + 0.5; x < x_max; x += 1.0) {
            // Create circular buffer around point
            BoostPoint center(x, y);
            BoostPolygon circle;
            double radius = sensor_range_ + 1.0 / std::sqrt(2.0);
            
            // Create circle using buffer operation
            bg::strategy::buffer::distance_symmetric<double> distance_strategy(radius);
            bg::buffer(center, circle, distance_strategy);

            // Find segments visible from this region
            std::vector<Segment> visible_segments;
            for (const auto& segment : map_segments_) {
                BoostLine line;
                line.push_back(BoostPoint(segment.first.x, segment.first.y));
                line.push_back(BoostPoint(segment.second.x, segment.second.y));

                // Check if line intersects but doesn't just touch circle
                if (bg::intersects(line, circle) && !bg::touches(line, circle)) {
                    visible_segments.push_back(segment);
                }
            }

            auto [r, c] = xyToRC({x, y});
            if (r >= 0 && r < map_rows && c >= 0 && c < map_cols) {
                region_segments[r][c] = std::move(visible_segments);
            }
        }
    }

    return region_segments;
}

std::pair<int, int> Map::xyToRC(const Point& xy) const {
    int x = static_cast<int>(std::floor(xy.x));
    int y = static_cast<int>(std::ceil(xy.y));

    int row = std::max(0, static_cast<int>(YR_ - y));
    int col = std::max(0, static_cast<int>(x + XC_));

    return std::make_pair(row, col);
}

// Helper function to create a Boost polygon from our data structures
BoostPolygon Map::createBoostPolygon(const std::vector<Point>& boundary, 
                                    const std::vector<std::vector<Point>>& holes) const {
    BoostPolygon poly;
    
    // Add exterior ring (boundary)
    for (const auto& pt : boundary) {
        bg::append(poly, BoostPoint(pt.x, pt.y));
    }
    // Close the ring if not already closed
    if (boundary.front().x != boundary.back().x || boundary.front().y != boundary.back().y) {
        bg::append(poly, BoostPoint(boundary.front().x, boundary.front().y));
    }
    
    // Add interior rings (holes)
    for (const auto& hole : holes) {
        typename BoostPolygon::inner_container_type::value_type inner_ring;
        
        for (const auto& pt : hole) {
            bg::append(inner_ring, BoostPoint(pt.x, pt.y));
        }
        // Close the ring if not already closed
        if (hole.front().x != hole.back().x || hole.front().y != hole.back().y) {
            bg::append(inner_ring, BoostPoint(hole.front().x, hole.front().y));
        }
        
        poly.inners().push_back(inner_ring);
    }
    
    return poly;
}

// Create a buffered polygon (expand/contract by distance)
BoostPolygon Map::createBufferedPolygon(const BoostPolygon& poly, double distance) const {
    BoostPolygon result;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
    bg::buffer(poly, result, distance_strategy);
    return result;
}

Map::~Map() {
    if (intersect_handle_ != nullptr) {
#ifdef _WIN32
        FreeLibrary((HMODULE)intersect_handle_);
#else
        dlclose(intersect_handle_);
#endif
    }
}

int main() {
    std::string map_name = "project";
    std::filesystem::path map_path = std::filesystem::absolute(
        std::filesystem::path(__FILE__).parent_path().parent_path() / "maps" / (map_name + ".json")
    );

    Map m(map_path.string(), 1.0);

    m.show(map_name, 1, false, {8.0, 8.0}, true);
    m.showRegions(map_name, 2, true, {8.0, 8.0}, true);

    return 0;
}