package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class constDrivetrain {
        public static final int joystickPort = 0;
        public static final double maxAngularRate = 0.4;
        public static final double deadbandPercent = 0.1;

        // Advanced Drive Control Constants
        public static final double deadband = 0.1;
        public static final double halfSpeedFactor = 0.35;
        public static final double rotationActiveTimeout = 0.1; // seconds
        public static final double rotationActiveThresholdDegrees = 5.0; // degrees
        public static final double inputCurve = 3.0; // Input exponent (1.0 = linear, 2.0 = squared, etc.)

        // Speed Control Constants
        public static final double maxSpeed = Units.feetToMeters(15); 
        public static final double speedModifier = 0.8; 

        // Dimensions
        public static final double chassisWidth = Units.inchesToMeters(27.0);
        public static final double chassisLength = Units.inchesToMeters(27.0);
    }
    public class constVision {
        
        // Basic filtering thresholds
        public static final double maxAmbiguity = 0.4;
        public static final double maxZError = 0.3; // Meters

        // Tag filtering for auto-alignment
        public static final double maxTagDistance = 5.0; // Maximum distance to consider tags (meters)
        public static final int[] blueTagIds = { 17, 18, 19, 20, 21, 22 };
        public static final int[] redTagIds = { 6, 7, 8, 9, 10, 11 };

        public enum StationTag {
            BLUE_LEFT(13),
            BLUE_RIGHT(12),
            RED_LEFT(1),
            RED_RIGHT(2);

            public final int id;

            StationTag(int id) {
                this.id = id;
            }
        }

        // Triangle detection parameters for auto-alignment
        public static final double detectionTriangleAngleDegrees = 120.0; // Angle of triangle detection zones for tag
                                                                          // selection

        // Tag selection scoring weights (all values represent penalty/bonus in meters)
        public static final double distanceWeight = 2.0; // Weight for distance component (1.0 = 1 meter = 1 point)
        public static final double skewWeight = 3.5; // Weight for tag orientation penalty (higher = more bias towards
                                                     // front-facing tags)
        public static final double ambiguityWeight = 0.25; // Weight for ambiguity penalty (higher = more penalty for
                                                           // uncertain detections)
        public static final double multiTagWeight = 0; // Weight for multi-tag bonus (negative = bonus, positive =
                                                       // penalty)

        // Standard deviation baselines, for 1 meter distance and 1 tag
        public static final double linearStdDevBaseline = 0.01; // Meters
        public static final double angularStdDevBaseline = 5.0; // Degrees

        // The layout of the AprilTags on the field

        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);

        // Camera names, must match names configured on coprocessor
        public static final String camera0Name = "camera_0";
        public static final String camera1Name = "camera_1";

        // Robot to camera transforms - SIMPLIFIED for testing
        // Left camera (camera_0): Front-left of robot
        // Right camera (camera_1): Front-right of robot
        // Using simple positions for testing - 12" forward, ±12" sideways, 9.3" up
        public static final Transform3d robotToCamera0 = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45))); // Look forward-left

        public static final Transform3d robotToCamera1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45))); // Look forward-right

        // Standard deviation multipliers for each camera
        public static final double[] cameraStdDevFactors = new double[] {
                2.0, // Camera 0
                2.0 // Camera 1
        };

        // Multipliers to apply for MegaTag 2 observations
        public static final double linearStdDevMegatag2Factor = 0.5;
        public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

        // Camera simulation properties
        public static final int cameraFPS = 30;
        public static final int cameraResolutionWidth = 800;
        public static final int cameraResolutionHeight = 600;
        public static final double cameraFOVDegrees = 70.0;
    }
    
    public class constTurret {
        public static final int turretYawMotorID = 10;

        // PID gains for turret yaw control (Phoenix 6 PositionVoltage)
        public static final double pYaw = 25.0;  // Proportional gain - increased from 1
        public static final double iYaw = 0.0;   // Integral gain
        public static final double dYaw = 0.5;   // Derivative gain for damping
    }
}
