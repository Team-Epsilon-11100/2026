package frc.robot.utils;

/**
 * Utility class for calculating launch angles based on projectile motion physics.
 * Uses kinematic equations to determine the optimal angle for a given velocity,
 * distance, and height difference.
 */
public class FindLaunchAngle {
    
    // Gravitational acceleration constant (m/s²)
    private static final double GRAVITY = 9.8;
    
    /**
     * Calculates the launch angle needed to hit a target using projectile motion equations.
     * 
     * This uses the quadratic formula solution for projectile motion:
     * tan(θ) = (d + √(d² - (2g·d²/v²)·((g·d²/v²) - h + target_height))) / ((g·d²)/v²)
     * 
     * @param velocity Launch velocity in meters per second (m/s)
     * @param distance Horizontal distance to target in meters (m)
     * @param shooterHeight Starting height of the shooter in meters (m)
     * @param targetHeight Height of the target in meters (m) - defaults to 1.0m if using 3-param method
     * @return Launch angle in degrees (0° to 90°)
     * @throws IllegalArgumentException if no valid solution exists (target unreachable with given velocity)
     */
    public static double calculateLaunchAngle(
            double velocity,
            double distance,
            double shooterHeight,
            double targetHeight
    ) {
        double v2 = velocity * velocity;
        double d2 = distance * distance;
        
        // Height difference (negative if shooting upward to target)
        double heightDiff = shooterHeight - targetHeight;
        
        // Discriminant of the quadratic formula
        // If negative, no real solution exists (target is out of range)
        double radical = d2 - (2 * GRAVITY * d2 / v2) * ((GRAVITY * d2 / (2 * v2)) + heightDiff);
        
        if (radical < 0) {
            throw new IllegalArgumentException(
                String.format(
                    "No valid launch angle exists. Target unreachable with velocity %.2f m/s at distance %.2f m",
                    velocity, distance
                )
            );
        }
        
        // Numerator: choose the + solution for the lower angle (more reliable)
        double numerator = distance + Math.sqrt(radical);
        
        // Denominator
        double denominator = (GRAVITY * d2) / v2;
        
        if (denominator == 0) {
            throw new IllegalArgumentException("Invalid calculation: denominator is zero");
        }
        
        // Calculate angle in radians, then convert to degrees
        double angleRadians = Math.atan(numerator / denominator);
        double angleDegrees = Math.toDegrees(angleRadians);
        
        return angleDegrees;
    }
    
    /**
     * Calculates the launch angle with a default target height of 1.0 meter.
     * Convenience method for when target height is not specified.
     * 
     * @param velocity Launch velocity in meters per second (m/s)
     * @param distance Horizontal distance to target in meters (m)
     * @param shooterHeight Starting height of the shooter in meters (m)
     * @return Launch angle in degrees (0° to 90°)
     * @throws IllegalArgumentException if no valid solution exists
     */
    public static double calculateLaunchAngle(
            double velocity,
            double distance,
            double shooterHeight
    ) {
        return calculateLaunchAngle(velocity, distance, shooterHeight, 1.0);
    }
    
    /**
     * Test method to verify launch angle calculations
     */
    public static void main(String[] args) {
        System.out.println("=== Launch Angle Calculator Test ===\n");
        
        // Test case 1: Example from original code
        double velocity1 = 8.6;
        double distance1 = 5.37;
        double shooterHeight1 = 0.135;
        double angle1 = calculateLaunchAngle(velocity1, distance1, shooterHeight1);
        System.out.printf("Test 1 - Velocity: %.2f m/s, Distance: %.2f m, Height: %.2f m\n", 
                         velocity1, distance1, shooterHeight1);
        System.out.printf("Launch Angle: %.2f degrees\n\n", angle1);
        
        // Test case 2: Close range shot
        double velocity2 = 10.0;
        double distance2 = 2.0;
        double shooterHeight2 = 0.5;
        double angle2 = calculateLaunchAngle(velocity2, distance2, shooterHeight2);
        System.out.printf("Test 2 - Velocity: %.2f m/s, Distance: %.2f m, Height: %.2f m\n", 
                         velocity2, distance2, shooterHeight2);
        System.out.printf("Launch Angle: %.2f degrees\n\n", angle2);
        
        // Test case 3: Long range shot
        double velocity3 = 15.0;
        double distance3 = 8.0;
        double shooterHeight3 = 0.2;
        double angle3 = calculateLaunchAngle(velocity3, distance3, shooterHeight3);
        System.out.printf("Test 3 - Velocity: %.2f m/s, Distance: %.2f m, Height: %.2f m\n", 
                         velocity3, distance3, shooterHeight3);
        System.out.printf("Launch Angle: %.2f degrees\n\n", angle3);
        
        // Test case 4: Unreachable target (should throw exception)
        try {
            double velocity4 = 5.0;
            double distance4 = 10.0;
            double shooterHeight4 = 0.5;
            double angle4 = calculateLaunchAngle(velocity4, distance4, shooterHeight4);
            System.out.printf("Test 4 - Velocity: %.2f m/s, Distance: %.2f m, Height: %.2f m\n", 
                             velocity4, distance4, shooterHeight4);
            System.out.printf("Launch Angle: %.2f degrees\n\n", angle4);
        } catch (IllegalArgumentException e) {
            System.out.println("Test 4 - Expected failure: " + e.getMessage() + "\n");
        }
    }
}
