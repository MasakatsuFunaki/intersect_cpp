# intersect_cpp

Prebuilt binaries for a fast, dependency-free **2D geometry intersection**
library for C++. Download the bundle for your platform, link the library, and
include a single header.

## What's inside

The library provides boolean overlap tests and point/segment/line
intersections for a wide set of 2D primitives:

- Points, segments, lines, rays
- Circles, triangles, quads, axis-aligned boxes (AABB)
- General simple polygons (convex or concave)
- Circular arcs, annuli (rings), capsules (stadiums), ellipses

All operations are exposed through one class, `geometry::GeometryFacade`, in
`geometry_facade.h`.

## Download

Grab the folder for your platform from [`release/`](release/) or a tagged
[release](../../releases):

| Platform | Files |
|----------|-------|
| Windows  | `geometry.dll`, `geometry.lib`, `include/geometry_facade.h` |
| Linux    | `libgeometry.so`, `include/geometry_facade.h` |
| macOS    | `libgeometry.dylib`, `include/geometry_facade.h` |

## Usage

```cpp
#include "geometry_facade.h"

int main() {
    geometry::GeometryFacade g;

    geometry::Circle c{0.0, 0.0, 2.0};        // {cx, cy, r}
    geometry::Aabb    box{1.0, 1.0, 3.0, 3.0}; // {xmin, ymin, xmax, ymax}

    bool hit = g.circle_quad_intersect(
        c, {1,1, 3,1, 3,3, 1,3});             // quad = 4 vertices in order
    (void)hit;
    return 0;
}
```

Link against the import library / shared object for your platform. See the
header for the full API and the exact numeric layout of each shape.

### Compatibility note

These are C++ shared libraries whose public interface uses standard-library
types. Build your application with a C++17 (or newer) toolchain compatible
with the one that produced the binaries (on Windows, a matching MSVC runtime).

## License

See [LICENSE](LICENSE).
