# Nudge

A high-performance, statically-linked 3D physics library with a self-contained mathematics system.

## Overview

Nudge provides a complete solution for 3D physics simulation with an integrated mathematical foundation. Designed for performance and ease of integration, the library requires no external dependencies and links statically into your applications.

## Features

- **3D Physics Simulation**: Complete rigid body dynamics, collision detection, and response
- **Self-Contained Mathematics**: Built-in vector, matrix, and quaternion operations
- **Static Linking**: No runtime dependencies, easy deployment
- **High Performance**: Optimized algorithms for real-time applications
- **Modern C++**: Built with C++20 for optimal performance and developer experience
- **Cross-Platform**: Compatible with Windows, macOS, and Linux

## Quick Start

### Installation

1. Clone the repository:
```bash
git clone https://github.com/JamesMillsDev/nudge.git
cd nudge
```

2. Build the library:
```bash
mkdir build
cd build
cmake ..
make
```

3. Link against the static library in your project:
```cmake
target_link_libraries(your_project nudge)
```

### Basic Usage

```cpp
#include "nudge/physics.h"

int main() {
    // Initialize physics world
    nudge::PhysicsWorld world;
    
    // Create a rigid body
    nudge::RigidBody body;
    body.setPosition(nudge::Vector3(0.0f, 10.0f, 0.0f));
    body.setMass(1.0f);
    
    // Add to world
    world.addRigidBody(&body);
    
    // Simulation loop
    while (running) {
        world.step(deltaTime);
        
        // Get updated position
        auto position = body.getPosition();
        // Update your rendering...
    }
    
    return 0;
}
```

## API Reference

### Mathematics

- `nudge::Vector2` - 2D vector operations
- `nudge::Vector3` - 3D vector operations
- `nudge::Vector4` - 4D vector operations
- `nudge::Matrix2x2` - 2x2 matrix operations
- `nudge::Matrix3x3` - 3x3 matrix operations
- `nudge::Matrix4x4` - 4x4 transformation matrices
- `nudge::Quaternion` - Rotation representation

## Requirements

- C++20 compatible compiler
- CMake 3.16 or higher

## Performance

- Optimized broad-phase collision detection
- SIMD-accelerated mathematics operations
- Memory-efficient data structures
- Configurable simulation parameters

## License

This project is licensed under the GNU General Public License - see the [LICENSE](LICENSE) file for details.

## Support

- Create an issue for bug reports or feature requests
- Check existing issues before creating new ones
- Provide minimal reproduction cases for bugs

---

**Note**: Nudge is designed for applications requiring deterministic physics simulation with minimal external dependencies.
