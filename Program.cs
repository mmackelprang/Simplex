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
    
    public class SimplexOptimizer
    {
        private const double ALPHA = 1.0;    // Reflection coefficient
        private const double GAMMA = 2.0;    // Expansion coefficient
        private const double RHO = 0.5;      // Contraction coefficient
        private const double SIGMA = 0.5;    // Shrink coefficient
        private const double TOLERANCE = 1e-8;
        private const int MAX_ITERATIONS = 1000;
        
        public OptimizationResult OptimizeRegistration(List<Vector3> sourcePoints, List<Vector3> targetPoints)
        {
            // Initialize simplex with 7 vertices for 6D optimization (3 translation + 3 rotation)
            var simplex = InitializeSimplex();
            
            for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
            {
                // Evaluate all vertices
                var errors = new double[simplex.Length];
                for (int i = 0; i < simplex.Length; i++)
                {
                    errors[i] = EvaluateError(simplex[i], sourcePoints, targetPoints);
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
                    return CreateResult(simplex[bestIdx], errors[bestIdx], iteration + 1);
                }
                
                // Compute centroid (excluding worst point)
                var centroid = new double[6];
                for (int i = 0; i < simplex.Length; i++)
                {
                    if (i != worstIdx)
                    {
                        for (int j = 0; j < 6; j++)
                        {
                            centroid[j] += simplex[i][j];
                        }
                    }
                }
                for (int j = 0; j < 6; j++)
                {
                    centroid[j] /= (simplex.Length - 1);
                }
                
                // Reflection
                var reflected = Reflect(centroid, simplex[worstIdx]);
                var reflectedError = EvaluateError(reflected, sourcePoints, targetPoints);
                
                if (reflectedError < errors[bestIdx])
                {
                    // Expansion
                    var expanded = Expand(centroid, reflected);
                    var expandedError = EvaluateError(expanded, sourcePoints, targetPoints);
                    
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
                    
                    var contractedError = EvaluateError(contracted, sourcePoints, targetPoints);
                    
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
                                for (int j = 0; j < 6; j++)
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
                finalErrors[i] = EvaluateError(simplex[i], sourcePoints, targetPoints);
                if (finalErrors[i] < finalErrors[finalBestIdx]) finalBestIdx = i;
            }
            
            return CreateResult(simplex[finalBestIdx], finalErrors[finalBestIdx], MAX_ITERATIONS);
        }
        
        private double[][] InitializeSimplex()
        {
            // Create 7 vertices for 6D parameter space (tx, ty, tz, rx, ry, rz)
            var simplex = new double[7][];
            var initialStep = 0.1;
            
            // First vertex at origin
            simplex[0] = new double[] { 0, 0, 0, 0, 0, 0 };
            
            // Other vertices offset along each dimension
            for (int i = 1; i < 7; i++)
            {
                simplex[i] = new double[6];
                simplex[i][i - 1] = initialStep;
            }
            
            return simplex;
        }
        
        private double EvaluateError(double[] parameters, List<Vector3> sourcePoints, List<Vector3> targetPoints)
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
            var reflected = new double[6];
            for (int i = 0; i < 6; i++)
            {
                reflected[i] = centroid[i] + ALPHA * (centroid[i] - worst[i]);
            }
            return reflected;
        }
        
        private double[] Expand(double[] centroid, double[] reflected)
        {
            var expanded = new double[6];
            for (int i = 0; i < 6; i++)
            {
                expanded[i] = centroid[i] + GAMMA * (reflected[i] - centroid[i]);
            }
            return expanded;
        }
        
        private double[] Contract(double[] centroid, double[] point)
        {
            var contracted = new double[6];
            for (int i = 0; i < 6; i++)
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
