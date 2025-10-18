# Flexible Simplex Optimization Framework

A modern C# implementation of the Nelder-Mead Simplex algorithm with a flexible interface that supports various optimization scenarios, from 3D point registration to polynomial curve fitting.

## Overview

This application implements the Nelder-Mead Simplex optimization method with a flexible, delegate-based error function interface. The core optimizer can be used for:

- **3D Point Cloud Registration** - Translation and rotation alignment
- **2D Curve Fitting** - Linear, quadratic, or polynomial functions
- **Translation-Only Fitting** - 3D alignment without rotation
- **Custom Optimization Problems** - Any parametric optimization task

The key feature is a flexible `ErrorFunction` delegate that accepts any number of parameters, making the optimizer adaptable to different problem types.

## Algorithm Logic

### Nelder-Mead Simplex Method

The Nelder-Mead algorithm is a derivative-free optimization method that works by maintaining a simplex (a geometric shape with n+1 vertices in n-dimensional space). The algorithm automatically adjusts to the parameter count - for a 6-parameter problem (3 translation + 3 rotation), it uses a 7-vertex simplex in 6D space; for a 2-parameter linear fit, it uses a 3-vertex simplex in 2D space.

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
Root Mean Square (RMS) distance between transformed source points and their nearest target points:
```
RMS_Error = sqrt(sum((min_j ||P_transformed_i - P_target_j||²)) / N)
```

**Nearest Neighbor Matching:**
When source and target point counts differ, each transformed source point is matched to its nearest target point during optimization. This allows the algorithm to handle:
- More target points than source points (over-determined system)
- More source points than target points (under-determined system)
- Any arbitrary combination of point counts

## Flexible Interface

The optimizer uses a flexible `ErrorFunction` delegate interface that makes no assumptions about the problem being solved:

```csharp
public delegate double ErrorFunction(double[] parameters);
```

This allows the same optimizer to handle:

### 1. 3D Registration (Translation + Rotation)
6 parameters: `[tx, ty, tz, rx, ry, rz]`

### 2. Linear Function Fitting
2 parameters: `[slope, intercept]` for `y = a*x + b`

### 3. Polynomial Fitting
n+1 parameters for nth order polynomial: `[a0, a1, a2, ..., an]` for `y = a0 + a1*x + a2*x^2 + ... + an*x^n`

### 4. Translation-Only Fitting
3 parameters: `[tx, ty, tz]` for 3D alignment without rotation

### Example Usage

```csharp
// Example: Fit linear function to 2D data
var optimizer = new SimplexOptimizer();
var data = new List<(double x, double y)> { (1, 2), (2, 4), (3, 6) };

ErrorFunction errorFunc = (parameters) =>
{
    double slope = parameters[0];
    double intercept = parameters[1];
    
    double totalError = 0;
    foreach (var point in data)
    {
        double predicted = slope * point.x + intercept;
        double error = predicted - point.y;
        totalError += error * error;
    }
    return Math.Sqrt(totalError / data.Count);
};

var result = optimizer.Optimize(errorFunc, 2);  // 2 parameters
Console.WriteLine($"Linear fit: y = {result[0]:F3}*x + {result[1]:F3}");
```

Helper methods are provided in `FittingHelpers` class for common scenarios:
- `FitLinear()` - Linear function fitting
- `FitPolynomial()` - Polynomial of any order
- `FitTranslationOnly()` - 3D translation without rotation

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

The application supports two input modes:

#### Mode 1: Paired Points (Single File)
```bash
# Basic usage with paired points in one file
dotnet run <input_csv_file> [output_csv_file]

# Examples
dotnet run --project src/Simplex.csproj example1.input example1.output example1_results.csv
dotnet run --project src/Simplex.csproj example2.input example2.output
dotnet run --project src/Simplex.csproj example3.input example3.output results.csv
```

#### Mode 2: Separate Source and Target Files
```bash
# Usage with separate source and target files
dotnet run <source_csv_file> <target_csv_file> <output_csv_file>

# Example with different point counts
dotnet run source_points.csv target_points.csv output.csv
```

## Input File Format

### Paired Points Format (Single File)
CSV file with header and 6 columns per row:
```csv
x,y,z
1.0,2.0,3.0
2.0,2.0,3.0
...
```

### Separate Files Format
Source points file (3 columns):
```csv
x,y,z
1.0,2.0,3.0
4.0,5.0,6.0
...
```

Target points file (3 columns):
```csv
x,y,z
1.1,2.1,3.1
4.2,5.2,6.2
7.3,8.3,9.3
...
```

**Key Features:**
- **Flexible Point Counts**: Source and target point sets can have different sizes
- **Nearest Neighbor Matching**: When using separate files, the algorithm automatically matches each transformed source point to its nearest target point
- Minimum 3 non-colinear source points recommended for unique solution
- More points generally provide better accuracy
- Points should be well-distributed in 3D space

## Output File Format

CSV file with optimization results:
```csv
source_x,source_y,source_z,transformed_x,transformed_y,transformed_z,nearest_target_x,nearest_target_y,nearest_target_z,error
1.0,2.0,3.0,1.098,2.099,3.102,1.100,2.100,3.100,0.003
...
```

**Columns:**
- `source_*`: Original source points
- `transformed_*`: Source points after applying optimal transformation
- `nearest_target_*`: Nearest target point to the transformed source point
- `error`: Point-wise distance error to nearest target after transformation

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
