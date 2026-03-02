package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double intakeRpm = 5500;
    public static final double indexerRpm = 5500;
    public static final Pose3d hubGoal = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    public class constAutoAim {
        // Testing mode: aim directly at AprilTag center
        // Competition mode: aim at custom goal height
        public static final boolean useTagCenterForTesting = true; // ✅ Set to false for competition
        
        // Absolute goal height (used when useTagCenterForTesting = false)
        public static final double absoluteGoalHeightMeters = Units.inchesToMeters(72); // 72" off ground
    }
    
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

        // Left camera (camera_0): Front-left of robot
        // Right camera (camera_1): Front-right of robot
        // Using simple positions for testing - 12" forward, ±12" sideways, 9.3" up
        public static final Transform3d mainCameraOffset = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45))); // Look forward-left

        public static final Transform3d leftCameraOffset = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45))); // Look forward-right

        public static final Transform3d rightCameraOffset = new Transform3d(
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
        public static final int cameraResolutionWidth = 640;
        public static final int cameraResolutionHeight = 480;
        public static final double cameraFOVDegrees = 70.0;
    }
    
    public class constHood {
        public static final int hoodMotorId = 31; // 3x = Shooter system
        public static final double maxHoodAngleDegrees = 56;
        public static final double minHoodAngleDegrees = 20.0;
        public static final double maxHoodMotorPos = 12.5;
        public static final double minHoodMotorPos = 0.5;

        public static final double angleToPosFactor =
            (maxHoodMotorPos - minHoodMotorPos) /
            (maxHoodAngleDegrees - minHoodAngleDegrees);

        

        public static double kP = 50.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
    }

    public class constIntake {
        public static final int intakeMotorId = 41; // 4x = Intake system
        public static final int pivotMotorId = 42;

        public static final double intakeRPM = 5500;
        
        public static final double intakeKp = 0.1;
        public static final double intakeKi = 0.0;
        public static final double intakeKd = 0.0;

        public static final double pivotKp = 0.1;
        public static final double pivotKi = 0.0;   
        public static final double pivotKd = 0.0;
    }

    public class constIndexer {
        public static final int indexerMotorId = 51; // 5x = Indexer system
        
        public static final double maxIndexerRPM = 3000;
        public static final double minIndexerRPM = 0;
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public class constKicker {
        public static final int kickerMotorId = 61; // 6x = Kicker system
        
        public static final double maxKickerRPM = 4000;
        public static final double minKickerRPM = 0;
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public class constFlywheel {
        public static final int flywheelMotorId = 33; // 3x = Shooter system (with Hood and Turret)
        public static final double maxFlywheelRPM = 6000;
        public static final double minFlywheelRPM = 0;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public class constBallisticSolver {
        // Shooter physical constants
        public static final double shooterHeightMeters = 0.135; // Height of shooter off ground (meters)
        public static final double gravity = 9.806; // m/s²
        
        // Flywheel configuration
        public static final double flywheelDiameterMeters = Units.inchesToMeters(4); // 4 inches
        public static final double exitVelocityFactor = 0.85; // Tune this: ball exit speed / wheel surface speed
        public static final double gearRatioMotorToWheel = 1.0; // Motor RPM / Wheel RPM (adjust for your robot)
        
        // Preferred shooting parameters
        public static final double preferredFlywheelRPM = 5500; // Target RPM for consistent shots
        public static final double preferredSpeedDeltaMps = 0.25; // Allow ±0.25 m/s from preferred speed
        
        // Speed limits (optional - set to null if no limits)
        public static final Double minSpeedMps = null; // Minimum exit speed (m/s)
        public static final Double maxSpeedMps = null; // Maximum exit speed (m/s)
        
        // Angle search resolution
        public static final double angleStepDeg = 0.05; // Search increment in degrees
    }
}