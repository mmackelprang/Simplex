# 3D Simplex Optimization for Point Registration

A modern C# implementation of the Nelder-Mead Simplex algorithm for 3D point registration, solving for optimal translation and rotation parameters to align two sets of 3D points.

## Overview

This application implements the Nelder-Mead Simplex optimization method to find the best-fit rigid body transformation (translation + rotation) between two sets of corresponding 3D points. It's particularly useful for:

- Point cloud registration
- Computer vision applications
- Robotics calibration
- Medical imaging alignment
- CAD/CAM applications

## Algorithm Logic

### Nelder-Mead Simplex Method

The Nelder-Mead algorithm is a derivative-free optimization method that works by maintaining a simplex (a geometric shape with n+1 vertices in n-dimensional space). For our 6-parameter optimization problem (3 translation + 3 rotation), we use a 7-vertex simplex in 6D space.

**Key Operations:**
1. **Reflection**: Reflect the worst point through the centroid
2. **Expansion**: If reflection is good, try expanding further
3. **Contraction**: If reflection fails, contract toward the centroid
4. **Shrinkage**: If contraction fails, shrink all points toward the best

**Parameters:**
- α = 1.0 (Reflection coefficient)
- γ = 2.0 (Expansion coefficient) 
- ρ = 0.5 (Contraction coefficient)
- σ = 0.5 (Shrink coefficient)

### 3D Transformation Model

The transformation applies:
1. **Rotation**: Using Euler angles (Rx, Ry, Rz) combined as rotation matrices
2. **Translation**: 3D vector (Tx, Ty, Tz)

**Transformation equation:**
```
P_transformed = R(Rx, Ry, Rz) * P_source + T(Tx, Ty, Tz)
```

**Error Function:**
Root Mean Square (RMS) distance between transformed source points and target points:
```
RMS_Error = sqrt(sum((||P_transformed_i - P_target_i||²)) / N)
```

## Requirements

- .NET 8.0 or later
- System.Numerics (included in .NET)

## Building and Running

### Clone and Build
```bash
git clone https://github.com/mmackelprang/Simplex.git
cd Simplex
dotnet build src/Simplex.csproj
```

### Run the Application
```bash
# Basic usage
dotnet run --project src/Simplex.csproj <source_input_file> <target_output_file> [results_csv_file]

# Examples
dotnet run --project src/Simplex.csproj example1.input example1.output example1_results.csv
dotnet run --project src/Simplex.csproj example2.input example2.output
dotnet run --project src/Simplex.csproj example3.input example3.output results.csv
```

## Input File Format

The application now uses separate input files for source and target points:

**Source Input File** (e.g., `example1.input`):
```csv
x,y,z
0.0,0.0,0.0
1.0,0.0,0.0
...
```

**Target Output File** (e.g., `example1.output`):
```csv
x,y,z
1.0,2.0,3.0
2.0,2.0,3.0
...
```

**Requirements:**
- Both files must have the same number of points
- Minimum 3 non-colinear point pairs for unique solution
- More points generally provide better accuracy
- Points should be well-distributed in 3D space

## Output File Format

CSV file with optimization results:
```csv
source_x,source_y,source_z,target_x,target_y,target_z,transformed_x,transformed_y,transformed_z,error
0.000000,0.000000,0.000000,1.000000,2.000000,3.000000,1.000000,2.000000,3.000000,0.000000
...
```

**Columns:**
- `source_*`: Original source points
- `target_*`: Target points to align to
- `transformed_*`: Source points after applying optimal transformation
- `error`: Point-wise distance error after transformation

## Example Usage Scenarios

### Example 1: Simple Translation
**Files:** `example1.input` and `example1.output`
- Simple translation offset with minimal rotation
- Demonstrates basic functionality
- Expected result: Pure translation with near-zero rotation

### Example 2: Complex Transformation
**Files:** `example2.input` and `example2.output`
- Combined translation and rotation
- More challenging optimization problem
- Tests algorithm robustness

### Example 3: Real-world Data
**Files:** `example3.input` and `example3.output`
- Realistic point cloud with noise
- Demonstrates practical application
- Shows convergence with imperfect data

## Understanding the Output

The application provides detailed console output:

```
3D Simplex Optimization for Point Registration
=============================================
Loaded 10 point pairs from example1_input.csv

Optimization completed in 45 iterations
Final error: 0.001234
Translation: (1.000, 2.000, 3.000)
Rotation (Euler angles): (0.100, 0.050, -0.075) radians
Results saved to example1_output.csv
```

**Key Metrics:**
- **Iterations**: Number of simplex iterations to convergence
- **Final Error**: RMS error of the optimal solution
- **Translation**: 3D translation vector (x, y, z)
- **Rotation**: Euler angles in radians (rotation about x, y, z axes)

## Algorithm Configuration

**Convergence Criteria:**
- Tolerance: 1e-8 (difference between best and worst simplex vertices)
- Maximum iterations: 1000

**Initial Simplex:**
- Origin: [0, 0, 0, 0, 0, 0]
- Step size: 0.1 for each parameter dimension

## Performance Considerations

- **Complexity**: O(n * iterations) where n is number of point pairs
- **Memory**: O(n) for point storage plus simplex vertices
- **Convergence**: Typically 50-200 iterations for well-conditioned problems
- **Robustness**: Handles local minima better than gradient-based methods

## Limitations and Best Practices

**Limitations:**
- Assumes rigid body transformation (no scaling or shearing)
- Requires corresponding point pairs
- Can be slower than analytical solutions for simple cases
- May converge to local minima with very poor initial guesses

**Best Practices:**
- Use at least 4-6 well-distributed point pairs
- Avoid colinear or coplanar point configurations
- Check final error to validate solution quality
- For large datasets, consider subsampling for initial estimate

## Theory and References

The Nelder-Mead simplex method was developed by John Nelder and Roger Mead in 1965. It's particularly well-suited for:
- Non-smooth optimization problems
- Functions where derivatives are unavailable or expensive
- Low to moderate dimensional problems (like our 6D case)

**Key advantages:**
- No gradient computation required
- Robust to noise in objective function
- Simple to implement and understand
- Good empirical performance for many problems

## Architecture and Extensibility

The project now uses an interface-based design to support different optimization algorithms:

### IOptimizationFunction Interface

```csharp
public interface IOptimizationFunction
{
    OptimizationResult Optimize(List<PointPair> points);
}
```

This interface allows easy swapping of optimization algorithms. The current implementation uses the Nelder-Mead Simplex method, but you can easily create alternative implementations (e.g., Particle Swarm Optimization, Genetic Algorithms, Gradient Descent) by implementing this interface.

**Example of using a different optimizer:**
```csharp
// Use the Nelder-Mead simplex optimizer
IOptimizationFunction optimizer = new SimplexOptimizer();

// Or create and use a different optimizer
// IOptimizationFunction optimizer = new MyCustomOptimizer();

var result = optimizer.Optimize(points);
```

## File Structure

```
Simplex/
├── README.md              # This documentation
├── .gitignore             # Git ignore file for build artifacts
├── src/                   # Source code directory
│   ├── Simplex.csproj     # .NET project file
│   └── Program.cs         # Main application with all classes
├── example1.input         # Simple translation example - source points
├── example1.output        # Simple translation example - target points
├── example2.input         # Complex transformation example - source points
├── example2.output        # Complex transformation example - target points
├── example3.input         # Real-world noisy data example - source points
└── example3.output        # Real-world noisy data example - target points
```

## Contributing

Contributions are welcome! Areas for enhancement:
- Additional transformation models (affine, perspective)
- Performance optimizations for large datasets
- Alternative optimization algorithms (PSO, genetic algorithms)
- Visualization tools for results
- Robust error handling and validation

## License

MIT License - feel free to use in your projects.

## Contact

For questions, issues, or suggestions, please open an issue on GitHub.
