using System;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using System.Numerics;

namespace Simplex3D
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("3D Simplex Optimization for Point Registration");
            Console.WriteLine("=============================================");
            
            if (args.Length == 0)
            {
                Console.WriteLine("Usage: dotnet run <input_csv_file> [target_csv_file] [output_csv_file]");
                Console.WriteLine("Example 1 (paired points): dotnet run example1_input.csv example1_output.csv");
                Console.WriteLine("Example 2 (separate files): dotnet run source.csv target.csv output.csv");
                return;
            }
            
            string inputFile = args[0];
            string targetFile = args.Length > 1 ? args[1] : null;
            string outputFile = args.Length > 2 ? args[2] : (args.Length > 1 ? args[1] : "output.csv");
            
            try
            {
                List<Vector3> sourcePoints;
                List<Vector3> targetPoints;
                
                // Try to load as paired points first
                if (targetFile == null || !File.Exists(targetFile) || targetFile.EndsWith("output.csv") || targetFile == "output.csv")
                {
                    // Single file with paired points
                    var points = LoadPairedPoints(inputFile);
                    Console.WriteLine($"Loaded {points.Count} point pairs from {inputFile}");
                    sourcePoints = points.ConvertAll(p => p.Source);
                    targetPoints = points.ConvertAll(p => p.Target);
                    
                    // Adjust output file if needed
                    if (targetFile != null && (targetFile.EndsWith("output.csv") || targetFile == "output.csv"))
                    {
                        outputFile = targetFile;
                    }
                }
                else
                {
                    // Separate source and target files
                    sourcePoints = LoadSinglePointList(inputFile);
                    targetPoints = LoadSinglePointList(targetFile);
                    Console.WriteLine($"Loaded {sourcePoints.Count} source points from {inputFile}");
                    Console.WriteLine($"Loaded {targetPoints.Count} target points from {targetFile}");
                }
                
                // Perform 3D registration using Nelder-Mead simplex
                var optimizer = new SimplexOptimizer();
                var result = optimizer.OptimizeRegistration(sourcePoints, targetPoints);
                
                // Display results
                Console.WriteLine($"\nOptimization completed in {result.Iterations} iterations");
                Console.WriteLine($"Final error: {result.FinalError:F6}");
                Console.WriteLine($"Translation: ({result.Translation.X:F3}, {result.Translation.Y:F3}, {result.Translation.Z:F3})");
                Console.WriteLine($"Rotation (Euler angles): ({result.RotationEuler.X:F3}, {result.RotationEuler.Y:F3}, {result.RotationEuler.Z:F3}) radians");
                
                // Apply transformation and save results
                SaveResults(sourcePoints, targetPoints, result, outputFile);
                Console.WriteLine($"Results saved to {outputFile}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
        }
        
        static List<PointPair> LoadPairedPoints(string filename)
        {
            var points = new List<PointPair>();
            var lines = File.ReadAllLines(filename);
            
            for (int i = 1; i < lines.Length; i++) // Skip header
            {
                var parts = lines[i].Split(',');
                if (parts.Length >= 6)
                {
                    var source = new Vector3(
                        float.Parse(parts[0], CultureInfo.InvariantCulture),
                        float.Parse(parts[1], CultureInfo.InvariantCulture),
                        float.Parse(parts[2], CultureInfo.InvariantCulture)
                    );
                    var target = new Vector3(
                        float.Parse(parts[3], CultureInfo.InvariantCulture),
                        float.Parse(parts[4], CultureInfo.InvariantCulture),
                        float.Parse(parts[5], CultureInfo.InvariantCulture)
                    );
                    points.Add(new PointPair(source, target));
                }
            }
            
            return points;
        }
        
        static List<Vector3> LoadSinglePointList(string filename)
        {
            var points = new List<Vector3>();
            var lines = File.ReadAllLines(filename);
            
            for (int i = 1; i < lines.Length; i++) // Skip header
            {
                var parts = lines[i].Split(',');
                if (parts.Length >= 3)
                {
                    var point = new Vector3(
                        float.Parse(parts[0], CultureInfo.InvariantCulture),
                        float.Parse(parts[1], CultureInfo.InvariantCulture),
                        float.Parse(parts[2], CultureInfo.InvariantCulture)
                    );
                    points.Add(point);
                }
            }
            
            return points;
        }
        
        static void SaveResults(List<Vector3> sourcePoints, List<Vector3> targetPoints, OptimizationResult result, string filename)
        {
            using (var writer = new StreamWriter(filename))
            {
                writer.WriteLine("source_x,source_y,source_z,transformed_x,transformed_y,transformed_z,nearest_target_x,nearest_target_y,nearest_target_z,error");
                
                foreach (var source in sourcePoints)
                {
                    var transformed = result.Transform(source);
                    
                    // Find nearest target point
                    Vector3 nearestTarget = targetPoints[0];
                    float minDistance = Vector3.Distance(transformed, targetPoints[0]);
                    
                    for (int i = 1; i < targetPoints.Count; i++)
                    {
                        float distance = Vector3.Distance(transformed, targetPoints[i]);
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            nearestTarget = targetPoints[i];
                        }
                    }
                    
                    writer.WriteLine($"{source.X:F6},{source.Y:F6},{source.Z:F6}," +
                                   $"{transformed.X:F6},{transformed.Y:F6},{transformed.Z:F6}," +
                                   $"{nearestTarget.X:F6},{nearestTarget.Y:F6},{nearestTarget.Z:F6},{minDistance:F6}");
                }
            }
        }
    }
    
    public class PointPair
    {
        public Vector3 Source { get; set; }
        public Vector3 Target { get; set; }
        
        public PointPair(Vector3 source, Vector3 target)
        {
            Source = source;
            Target = target;
        }
    }
    
    public class OptimizationResult
    {
        public Vector3 Translation { get; set; }
        public Vector3 RotationEuler { get; set; }
        public Matrix4x4 RotationMatrix { get; set; }
        public double FinalError { get; set; }
        public int Iterations { get; set; }
        
        public Vector3 Transform(Vector3 point)
        {
            // Apply rotation then translation
            var rotated = Vector3.Transform(point, RotationMatrix);
            return rotated + Translation;
        }
    }
    
    // Delegate for flexible error function that can be customized for different fitting scenarios
    public delegate double ErrorFunction(double[] parameters);
    
    // Helper class for fitting 2D data to various functions
    public static class FittingHelpers
    {
        // Fit X,Y data to a linear function: y = a*x + b
        public static double[] FitLinear(List<(double x, double y)> data)
        {
            var optimizer = new SimplexOptimizer();
            
            ErrorFunction errorFunc = (parameters) =>
            {
                double a = parameters[0];  // slope
                double b = parameters[1];  // intercept
                
                double totalError = 0;
                foreach (var point in data)
                {
                    double predicted = a * point.x + b;
                    double error = predicted - point.y;
                    totalError += error * error;
                }
                return Math.Sqrt(totalError / data.Count);
            };
            
            return optimizer.Optimize(errorFunc, 2);  // 2 parameters: slope and intercept
        }
        
        // Fit X,Y data to a polynomial: y = a0 + a1*x + a2*x^2 + ... + an*x^n
        public static double[] FitPolynomial(List<(double x, double y)> data, int polynomialOrder)
        {
            var optimizer = new SimplexOptimizer();
            
            ErrorFunction errorFunc = (parameters) =>
            {
                double totalError = 0;
                foreach (var point in data)
                {
                    double predicted = 0;
                    for (int i = 0; i < parameters.Length; i++)
                    {
                        predicted += parameters[i] * Math.Pow(point.x, i);
                    }
                    double error = predicted - point.y;
                    totalError += error * error;
                }
                return Math.Sqrt(totalError / data.Count);
            };
            
            return optimizer.Optimize(errorFunc, polynomialOrder + 1);  // n+1 parameters for nth order polynomial
        }
        
        // Fit 3D points with translation only (no rotation)
        public static Vector3 FitTranslationOnly(List<Vector3> sourcePoints, List<Vector3> targetPoints)
        {
            var optimizer = new SimplexOptimizer();
            
            ErrorFunction errorFunc = (parameters) =>
            {
                var translation = new Vector3((float)parameters[0], (float)parameters[1], (float)parameters[2]);
                
                double totalError = 0;
                foreach (var source in sourcePoints)
                {
                    var transformed = source + translation;
                    
                    // Find nearest target point
                    float minDistance = float.MaxValue;
                    foreach (var target in targetPoints)
                    {
                        float distance = Vector3.Distance(transformed, target);
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                        }
                    }
                    
                    totalError += minDistance * minDistance;
                }
                return Math.Sqrt(totalError / sourcePoints.Count);
            };
            
            var result = optimizer.Optimize(errorFunc, 3);  // 3 parameters: tx, ty, tz
            return new Vector3((float)result[0], (float)result[1], (float)result[2]);
        }
    }
    
    public class SimplexOptimizer
    {
        private const double ALPHA = 1.0;    // Reflection coefficient
        private const double GAMMA = 2.0;    // Expansion coefficient
        private const double RHO = 0.5;      // Contraction coefficient
        private const double SIGMA = 0.5;    // Shrink coefficient
        private const double TOLERANCE = 1e-8;
        private const int MAX_ITERATIONS = 1000;
        
        public double[] Optimize(ErrorFunction errorFunction, int parameterCount, double initialStep = 0.1)
        {
            // Initialize simplex with parameterCount+1 vertices for parameterCount-dimensional optimization
            var simplex = InitializeSimplex(parameterCount, initialStep);
            
            for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
            {
                // Evaluate all vertices
                var errors = new double[simplex.Length];
                for (int i = 0; i < simplex.Length; i++)
                {
                    errors[i] = errorFunction(simplex[i]);
                }
                
                // Find best, worst, and second worst
                int bestIdx = 0, worstIdx = 0, secondWorstIdx = 0;
                for (int i = 1; i < errors.Length; i++)
                {
                    if (errors[i] < errors[bestIdx]) bestIdx = i;
                    if (errors[i] > errors[worstIdx])
                    {
                        secondWorstIdx = worstIdx;
                        worstIdx = i;
                    }
                    else if (errors[i] > errors[secondWorstIdx] && i != worstIdx)
                    {
                        secondWorstIdx = i;
                    }
                }
                
                // Check convergence
                if (Math.Abs(errors[worstIdx] - errors[bestIdx]) < TOLERANCE)
                {
                    return simplex[bestIdx];
                }
                
                // Compute centroid (excluding worst point)
                var centroid = new double[parameterCount];
                for (int i = 0; i < simplex.Length; i++)
                {
                    if (i != worstIdx)
                    {
                        for (int j = 0; j < parameterCount; j++)
                        {
                            centroid[j] += simplex[i][j];
                        }
                    }
                }
                for (int j = 0; j < parameterCount; j++)
                {
                    centroid[j] /= (simplex.Length - 1);
                }
                
                // Reflection
                var reflected = Reflect(centroid, simplex[worstIdx]);
                var reflectedError = errorFunction(reflected);
                
                if (reflectedError < errors[bestIdx])
                {
                    // Expansion
                    var expanded = Expand(centroid, reflected);
                    var expandedError = errorFunction(expanded);
                    
                    if (expandedError < reflectedError)
                    {
                        simplex[worstIdx] = expanded;
                    }
                    else
                    {
                        simplex[worstIdx] = reflected;
                    }
                }
                else if (reflectedError < errors[secondWorstIdx])
                {
                    simplex[worstIdx] = reflected;
                }
                else
                {
                    // Contraction
                    double[] contracted;
                    if (reflectedError < errors[worstIdx])
                    {
                        // Outside contraction
                        contracted = Contract(centroid, reflected);
                    }
                    else
                    {
                        // Inside contraction
                        contracted = Contract(centroid, simplex[worstIdx]);
                    }
                    
                    var contractedError = errorFunction(contracted);
                    
                    if (contractedError < Math.Min(reflectedError, errors[worstIdx]))
                    {
                        simplex[worstIdx] = contracted;
                    }
                    else
                    {
                        // Shrink all points toward best
                        for (int i = 0; i < simplex.Length; i++)
                        {
                            if (i != bestIdx)
                            {
                                for (int j = 0; j < parameterCount; j++)
                                {
                                    simplex[i][j] = simplex[bestIdx][j] + SIGMA * (simplex[i][j] - simplex[bestIdx][j]);
                                }
                            }
                        }
                    }
                }
            }
            
            // Return best result if max iterations reached
            var finalErrors = new double[simplex.Length];
            int finalBestIdx = 0;
            for (int i = 0; i < simplex.Length; i++)
            {
                finalErrors[i] = errorFunction(simplex[i]);
                if (finalErrors[i] < finalErrors[finalBestIdx]) finalBestIdx = i;
            }
            
            return simplex[finalBestIdx];
        }
        
        public OptimizationResult OptimizeRegistration(List<Vector3> sourcePoints, List<Vector3> targetPoints)
        {
            // Define the error function for 3D registration with translation and rotation
            ErrorFunction errorFunc = (parameters) => EvaluateRegistrationError(parameters, sourcePoints, targetPoints);
            
            // Optimize with 6 parameters: 3 translation + 3 rotation
            var bestParams = Optimize(errorFunc, 6);
            
            // Create result object with iteration count
            return CreateResult(bestParams, errorFunc(bestParams), MAX_ITERATIONS);
        }
        
        private double[][] InitializeSimplex(int parameterCount, double initialStep)
        {
            // Create parameterCount+1 vertices for parameterCount-dimensional parameter space
            var simplex = new double[parameterCount + 1][];
            
            // First vertex at origin
            simplex[0] = new double[parameterCount];
            
            // Other vertices offset along each dimension
            for (int i = 1; i <= parameterCount; i++)
            {
                simplex[i] = new double[parameterCount];
                simplex[i][i - 1] = initialStep;
            }
            
            return simplex;
        }
        
        private double EvaluateRegistrationError(double[] parameters, List<Vector3> sourcePoints, List<Vector3> targetPoints)
        {
            var translation = new Vector3((float)parameters[0], (float)parameters[1], (float)parameters[2]);
            var rotation = CreateRotationMatrix((float)parameters[3], (float)parameters[4], (float)parameters[5]);
            
            double totalError = 0;
            foreach (var source in sourcePoints)
            {
                var transformed = Vector3.Transform(source, rotation) + translation;
                
                // Find nearest target point
                float minDistance = float.MaxValue;
                foreach (var target in targetPoints)
                {
                    float distance = Vector3.Distance(transformed, target);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }
                
                totalError += minDistance * minDistance; // Sum of squared errors
            }
            
            return Math.Sqrt(totalError / sourcePoints.Count); // RMS error
        }
        
        private Matrix4x4 CreateRotationMatrix(float rx, float ry, float rz)
        {
            return Matrix4x4.CreateRotationX(rx) * Matrix4x4.CreateRotationY(ry) * Matrix4x4.CreateRotationZ(rz);
        }
        
        private double[] Reflect(double[] centroid, double[] worst)
        {
            var reflected = new double[centroid.Length];
            for (int i = 0; i < centroid.Length; i++)
            {
                reflected[i] = centroid[i] + ALPHA * (centroid[i] - worst[i]);
            }
            return reflected;
        }
        
        private double[] Expand(double[] centroid, double[] reflected)
        {
            var expanded = new double[centroid.Length];
            for (int i = 0; i < centroid.Length; i++)
            {
                expanded[i] = centroid[i] + GAMMA * (reflected[i] - centroid[i]);
            }
            return expanded;
        }
        
        private double[] Contract(double[] centroid, double[] point)
        {
            var contracted = new double[centroid.Length];
            for (int i = 0; i < centroid.Length; i++)
            {
                contracted[i] = centroid[i] + RHO * (point[i] - centroid[i]);
            }
            return contracted;
        }
        
        private OptimizationResult CreateResult(double[] parameters, double error, int iterations)
        {
            var translation = new Vector3((float)parameters[0], (float)parameters[1], (float)parameters[2]);
            var rotationEuler = new Vector3((float)parameters[3], (float)parameters[4], (float)parameters[5]);
            var rotationMatrix = CreateRotationMatrix(rotationEuler.X, rotationEuler.Y, rotationEuler.Z);
            
            return new OptimizationResult
            {
                Translation = translation,
                RotationEuler = rotationEuler,
                RotationMatrix = rotationMatrix,
                FinalError = error,
                Iterations = iterations
            };
        }
    }
}
