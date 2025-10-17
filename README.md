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
Root Mean Square (RMS) distance between transformed source points and their nearest target points:
```
RMS_Error = sqrt(sum((min_j ||P_transformed_i - P_target_j||²)) / N)
```

**Nearest Neighbor Matching:**
When source and target point counts differ, each transformed source point is matched to its nearest target point during optimization. This allows the algorithm to handle:
- More target points than source points (over-determined system)
- More source points than target points (under-determined system)
- Any arbitrary combination of point counts

## Requirements

- .NET 6.0 or later
- System.Numerics (included in .NET)

## Building and Running

### Clone and Build
```bash
git clone https://github.com/mmackelprang/Simplex.git
cd Simplex
dotnet build
```

### Run the Application

The application supports two input modes:

#### Mode 1: Paired Points (Single File)
```bash
# Basic usage with paired points in one file
dotnet run <input_csv_file> [output_csv_file]

# Examples
dotnet run example1_input.csv example1_output.csv
dotnet run example2_input.csv
dotnet run example3_input.csv results.csv
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
source_x,source_y,source_z,target_x,target_y,target_z
1.0,2.0,3.0,1.1,2.1,3.1
4.0,5.0,6.0,4.2,5.2,6.2
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
**File:** `example1_input.csv`
- Simple translation offset with minimal rotation
- Demonstrates basic functionality
- Expected result: Pure translation with near-zero rotation

### Example 2: Complex Transformation
**File:** `example2_input.csv`
- Combined translation and rotation
- More challenging optimization problem
- Tests algorithm robustness

### Example 3: Real-world Data
**File:** `example3_input.csv`
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

## File Structure

```
Simplex/
├── README.md              # This documentation
├── Program.cs             # Main application with all classes
├── example1_input.csv     # Simple translation example
├── example1_output.csv    # Expected output for example 1
├── example2_input.csv     # Complex transformation example
├── example2_output.csv    # Expected output for example 2
├── example3_input.csv     # Real-world noisy data example
└── example3_output.csv    # Expected output for example 3
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
