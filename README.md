# DoDeeDum

DoDeeDum computes 2D silhouettes and contours from 3D meshes with support for skeletal animation and skinning. Extract outline polygons, calculate projected areas, and analyze joint influence on character silhouettes from any viewpoint.

Perfect for procedural animation systems that need to measure visible joint influence, compute character coverage from camera views, or analyze skeletal deformation impact on mesh silhouettes.

Key capabilities: mesh projection, silhouette extraction, skinned mesh analysis, joint-weighted area calculation, contour detection, and skeletal animation support for procedural IK and constraint systems.

The name is intentionally stupid because it's stupid how computationally expensive it is to answer "how big does this look?"

## Weighted Area Calculation

### Conceptual Model

The weighted area calculation answers the question: "How much of this silhouette is influenced by a specific set of joints?"

Conceptually, it works as if you:
1. Rendered the projected mesh with alpha blending
2. Set each fragment's alpha to the sum of skinning weights for the selected joints, clamped to [0, 1]
3. Summed all resulting alpha values across the silhouette

However, DoDeeDum computes this **analytically** rather than through rasterization:
- Projects 3D triangles to 2D
- Calculates geometric area for each triangle
- Weights each triangle's area by its vertices' skinning influences
- Integrates across the entire silhouette

### Example

Consider a character's arm viewed from the side, and you want to know how much of the visible silhouette is controlled by the elbow joint:

```cpp
std::vector<uint32_t> elbowJoints = {ELBOW_JOINT_ID};
auto silhouettes = DoDeeDum::GetSilhouettes(
    mesh, 
    projectionMatrix, 
    DoDeeDum::GET_WEIGHTED_AREA,
    elbowJoints
);

// silhouettes[0].area: Total visible area (e.g., 1000 sq units)
// silhouettes[0].weightedArea: Area influenced by elbow (e.g., 650 sq units)
// Interpretation: 65% of the visible arm is controlled by the elbow joint
```

### Applications

Weighted areas are useful for:
- **Procedural Animation**: Determine which joints contribute most to a character's visible silhouette from a given viewpoint
- **IK Solving**: Prioritize joint chains based on their visual impact
- **Constraint Systems**: Weight constraints by the visibility and influence of affected joints
- **Animation Analysis**: Measure how joint movements affect the perceived shape

The second moment of weighted area extends this concept to rotational distribution, measuring how joint influence is spread across the silhouette's shape.

## Features

- **Silhouette Extraction**: Compute outlines, areas, and 2nd moment of area from projected 3D meshes
- **Skinning Support**: Filter geometry by joint influences for skeletal animation systems
- **Delaunay Triangulation**: Generate optimal triangulation of silhouette regions
- **Weighted Area Calculation**: Account for skinning weights in area computations
- **Flexible Projection**: Support for arbitrary projection matrices 
- **High Performance**: Optimized spatial queries using quadtree acceleration structures

## Usage

### Basic Silhouette Extraction

```cpp
#include <dodeedum.h>

// Project mesh using a projection matrix
std::vector<DoDeeDum::Silhouette> silhouettes =
    DoDeeDum::GetSilhouettes(mesh, projectionMatrix);
```

### Working with Joint Subsets

```cpp
// Get silhouettes influenced by specific joints
std::vector<uint32_t> joints = {0, 1, 5};
std::vector<DoDeeDum::Silhouette> silhouettes =
    DoDeeDum::GetSilhouettes(mesh, projectionMatrix, DoDeeDum::GET_ALL, joints);

// Each silhouette contains weighted area based on joint influences
for (const auto& sil : silhouettes) {
    std::cout << "Area: " << sil.area << "\n";
    std::cout << "Weighted Area: " << sil.weightedArea << "\n";
    std::cout << "Center: " << sil.center.x << ", " << sil.center.y << "\n";
}
```

### Selective Computation

Use `Options` flags to compute only what you need:

```cpp
using namespace DoDeeDum;

// Get only outline and area, skip expensive calculations
auto options = GET_OUTLINE | GET_AREA;
std::vector<Silhouette> silhouettes =
    GetSilhouettes(mesh, projectionMatrix, options);

// Available options:
// - GET_DELAUNY_PERIMITER: Compute Delaunay triangulation
// - GET_OUTLINE: Extract outline polygon
// - GET_AREA: Calculate area
// - GET_WEIGHTED_AREA: Calculate weighted area (requires skinning data)
// - GET_AREA_2ND_MOMENT: Calculate second moment of area
// - GET_CENTER_OF_AREA: Calculate centroid
// - GET_ALL: Compute everything
```

### Silhouette Data Structure

```cpp
struct Silhouette {
    std::vector<glm::vec2>  points;                // 2D projected points
    std::vector<uint32_t>   outline;               // Indices forming outline
    std::vector<glm::uvec3> delauny_triangulation; // Delaunay triangulation

    double area;            // Total area
    double weightedArea;    // Area weighted by skinning influences
    double area2ndMoment;   // Second moment of area
    double weightedArea2ndMoment;   // Second moment of area
    glm::vec2 center;       // Centroid
};
```

## Mesh Format

DoDeeDum works with a flexible mesh representation:

```cpp
struct Primitive {
    std::function<glm::dvec4(uint32_t)>   position;  // Position accessor
    std::function<glm::i64vec4(uint32_t)> joints;    // Joint indices
    std::function<glm::dvec4(uint32_t)>   weights;   // Joint weights
    MeshIndices indices;                             // Index buffer
    bool flipNormals; // does nothing (for expansion)
    bool isDoubleSided; // does nothing (for expansion)
    bool hasAlpha; // does nothing (for expansion)
};

using Mesh = std::vector<Primitive>;
```

The library supports various index formats (8/16/32-bit) and geometry types (triangles, strips, fans). And comes with built in accessors that imitate glVertexAttrib:

```cpp
enum class AttribType : uint16_t {
	BYTE           = 0x1400,  ///< Signed 8-bit integer
	UNSIGNED_BYTE  = 0x1401,  ///< Unsigned 8-bit integer
	SHORT          = 0x1402,  ///< Signed 16-bit integer
	UNSIGNED_SHORT = 0x1403,  ///< Unsigned 16-bit integer
	INT            = 0x1404,  ///< Signed 32-bit integer
	UNSIGNED_INT   = 0x1405,  ///< Unsigned 32-bit integer
	FLOAT          = 0x1406,  ///< 32-bit floating point
	DOUBLE         = 0x140A,  ///< 64-bit floating point
	HALF_FLOAT     = 0x140B,  ///< 16-bit floating point
};

struct MeshAttrib
{		
	void * src;
	uint64_t byteLength;
    uint32_t offset;
	AttribType type;
    uint16_t stride;
	uint8_t size;
    bool    normalized;
    
    size_t		 get_type_size() const;
	glm::dvec4   readf(uint32_t index) const;
	glm::i64vec4 readi(uint32_t index) const;
};
```

## Implementation Details

### Spatial Acceleration

DoDeeDum uses a cache-efficient quadtree implementation:
- Index-based nodes (no pointers) for better cache locality
- Bump allocator with pre-reserved memory
- Up to 4 triangles per leaf node
- Exploits sorted triangle data for faster construction

### Projection Pipeline

1. **Mesh Projection**: 3D vertices are projected to 2D using the provided transformation
2. **Weight Filtering**: Vertices are filtered based on joint influences
3. **Sorting**: Points and triangles are spatially sorted for efficient queries
4. **Silhouette Extraction**: Outlines are traced and geometric properties computed
5. **Triangulation**: Optional Delaunay triangulation of interior regions

## Dependencies

- **GLM**: For vector/matrix math
- **C++17**: Standard library features

## Building

DoDeeDum is a header/source library. Include the headers and link against the compiled sources:

```cmake
include_directories(dodeedum/include)
add_library(dodeedum
    dodeedum/src/dodeedum_project2d.cpp
    dodeedum/src/dodeedum_quadtree.cpp
    dodeedum/src/dodeedum_mesh.cpp
)
target_link_libraries(your_target dodeedum)
```

## License

MIT License - Copyright (c) 2025 Spehleon LP

See [LICENSE](LICENSE) for full details.
