#ifndef GEOMETRY_FACADE_H
#define GEOMETRY_FACADE_H

#include <array>
#include <memory>
#include <vector>

// Symbol visibility. Only the public API below is exported from the shared
// library; every internal helper is hidden, so the binary's export table
// exposes nothing but this interface. Consumers get the import form
// automatically (no macro to define). Define GEOMETRY_STATIC when linking a
// static build.
#if defined(_WIN32) || defined(__CYGWIN__)
#  if defined(GEOMETRY_STATIC)
#    define GEOMETRY_API
#  elif defined(GEOMETRY_BUILD_SHARED)
#    define GEOMETRY_API __declspec(dllexport)
#  else
#    define GEOMETRY_API __declspec(dllimport)
#  endif
#else
#  if defined(GEOMETRY_BUILD_SHARED)
#    define GEOMETRY_API __attribute__((visibility("default")))
#  else
#    define GEOMETRY_API
#  endif
#endif

namespace geometry {

class GeometryImpl; // Forward declaration

// Shape conventions (row-major, i.e. each shape's numbers listed row by row):
//   Point    {x, y}
//   Circle   {cx, cy, r}
//   Triangle {x1, y1, x2, y2, x3, y3}          vertices in order
//   Quad     {x1, y1, x2, y2, x3, y3, x4, y4}  vertices in order
//   Aabb     {xmin, ymin, xmax, ymax}
//   Polygon  vertices in order; simple (non-self-intersecting), convex or
//            concave, at most kMaxPolygonVertices vertices
//   Arc      {cx, cy, r, theta1, theta2}: counter-clockwise from theta1 to
//            theta2 (radians, wrap-aware; theta1 == theta2 is a full circle)
//   Annulus  {cx, cy, r_inner, r_outer}
//   Capsule  {x1, y1, x2, y2, r}: all points within r of the core segment
//   Ellipse  {cx, cy, a, b, theta}: semi-axes a, b, rotation theta (radians)
using Point = std::array<double, 2>;
using Circle = std::array<double, 3>;
using Triangle = std::array<double, 6>;
using Quad = std::array<double, 8>;
using Aabb = std::array<double, 4>;
using Polygon = std::vector<Point>;
using Arc = std::array<double, 5>;
using Annulus = std::array<double, 4>;
using Capsule = std::array<double, 5>;
using Ellipse = std::array<double, 5>;

// Upper bound baked into the library (bounded, fixed-capacity arrays; no
// dynamic allocation). Polygon arguments above this size are rejected.
constexpr std::size_t kMaxPolygonVertices = 128;

// Result of an intersection that can produce up to two points. The meaning
// of status depends on the function (see each method); unused point slots
// are filled with NaN.
struct IntersectionResult {
  double status;
  std::array<Point, 2> points;
};

class GEOMETRY_API GeometryFacade {
public:
  GeometryFacade();
  ~GeometryFacade();

  // Convex quad operations. quad_quad_intersect: separating axis test,
  // returns 1.0 when the quads overlap or touch, 0.0 otherwise (legacy
  // double return). is_rectangle: true if the 4 vertices form a
  // rectangle or square.
  double quad_quad_intersect(const Quad& quad1, const Quad& quad2);
  bool is_rectangle(const Quad& vertices);

  // Primitives
  // orient2d: +1 if c is left of a->b, 0 if collinear, -1 if right.
  double orient2d(const Point& a, const Point& b, const Point& c);
  Point closest_point_on_segment(const Point& p, const Point& a, const Point& b);

  // Point predicates (boundaries count as inside)
  bool point_on_segment(const Point& p, const Point& a, const Point& b);
  bool point_in_circle(const Point& p, const Circle& circle);
  bool point_in_triangle(const Point& p, const Triangle& tri);
  bool point_in_aabb(const Point& p, const Aabb& box);
  bool point_in_quad(const Point& p, const Quad& quad);

  // Linear x linear.
  // line_line_intersect:    status 0 = parallel, 1 = point, 2 = coincident.
  // segment_segment_intersect: status 0 = none, 1 = point,
  //                            2 = collinear overlap (points = endpoints).
  IntersectionResult line_line_intersect(const Point& p1, const Point& p2, const Point& q1, const Point& q2);
  IntersectionResult segment_segment_intersect(const Point& p1, const Point& p2, const Point& q1, const Point& q2);

  // Linear x circle: status = number of boundary crossings (1 = tangent).
  IntersectionResult line_circle_intersect(const Point& p1, const Point& p2, const Circle& circle);
  IntersectionResult segment_circle_intersect(const Point& p1, const Point& p2, const Circle& circle);

  // Circle x circle: status 0 = none, 1 = tangent, 2 = two points,
  //                  3 = coincident circles.
  IntersectionResult circle_circle_intersect(const Circle& circle1, const Circle& circle2);

  // Boolean overlap tests (touching counts as intersecting)
  bool circle_quad_intersect(const Circle& circle, const Quad& rect);
  bool circle_triangle_intersect(const Circle& circle, const Triangle& tri);
  bool triangle_triangle_intersect(const Triangle& tri1, const Triangle& tri2);
  bool aabb_aabb_intersect(const Aabb& box1, const Aabb& box2);

  // General simple polygons (convex or concave), boolean overlap tests.
  // Throw std::invalid_argument if a polygon exceeds kMaxPolygonVertices.
  bool point_in_polygon(const Point& p, const Polygon& polygon);
  bool line_polygon_intersect(const Point& p1, const Point& p2, const Polygon& polygon);
  bool segment_polygon_intersect(const Point& p1, const Point& p2, const Polygon& polygon);
  bool circle_polygon_intersect(const Circle& circle, const Polygon& polygon);
  bool polygon_polygon_intersect(const Polygon& poly1, const Polygon& poly2);

  // Circular arcs. Point-yielding intersections: status = number of
  // points; circle_arc/arc_arc report status 3 when the arc lies on a
  // coincident supporting circle (overlap, points = NaN).
  bool point_on_arc(const Point& p, const Arc& arc);
  IntersectionResult line_arc_intersect(const Point& p1, const Point& p2, const Arc& arc);
  IntersectionResult segment_arc_intersect(const Point& p1, const Point& p2, const Arc& arc);
  IntersectionResult circle_arc_intersect(const Circle& circle, const Arc& arc);
  IntersectionResult arc_arc_intersect(const Arc& arc1, const Arc& arc2);

  // Annulus (ring) overlap tests (touching counts as intersecting).
  bool point_in_annulus(const Point& p, const Annulus& annulus);
  bool segment_annulus_intersect(const Point& p1, const Point& p2, const Annulus& annulus);
  bool circle_annulus_intersect(const Circle& circle, const Annulus& annulus);

  // Capsules (stadium shapes) and the segment-distance primitive they
  // are built on (touching counts as intersecting).
  bool point_in_capsule(const Point& p, const Capsule& capsule);
  double segment_segment_distance(const Point& p1, const Point& p2, const Point& q1, const Point& q2);
  bool segment_capsule_intersect(const Point& p1, const Point& p2, const Capsule& capsule);
  bool circle_capsule_intersect(const Circle& circle, const Capsule& capsule);
  bool capsule_capsule_intersect(const Capsule& capsule1, const Capsule& capsule2);

  // Ellipses. Line/segment intersections are exact (affine reduction to
  // the unit circle); the boolean overlap tests use a robust
  // point-to-ellipse distance instead of solving the quartic system.
  bool point_in_ellipse(const Point& p, const Ellipse& ellipse);
  double point_ellipse_distance(const Point& p, const Ellipse& ellipse);
  IntersectionResult line_ellipse_intersect(const Point& p1, const Point& p2, const Ellipse& ellipse);
  IntersectionResult segment_ellipse_intersect(const Point& p1, const Point& p2, const Ellipse& ellipse);
  bool circle_ellipse_intersect(const Circle& circle, const Ellipse& ellipse);
  bool ellipse_ellipse_intersect(const Ellipse& ellipse1, const Ellipse& ellipse2);

private:
  std::unique_ptr<GeometryImpl> pimpl_;
};

} // namespace geometry

#endif // GEOMETRY_FACADE_H
