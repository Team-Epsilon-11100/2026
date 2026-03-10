package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {


   

    public class constAutoAim {
        // Testing mode: aim turret directly at closest AprilTag (no odometry logic)
        // Set to false for competition to use odometry + neutral zone ferry logic
        public static final boolean useTagYawForTesting = true;

        // Testing mode for elevation: aim hood/flywheel at AprilTag center height
        // Set to false for competition to use alliance HUB height
        public static final boolean useTagCenterForTesting = true;

        // HUB positions on the field (x, y, z) in meters
        public static final Translation3d blueHubPosition = new Translation3d(4.63,  4.04, 1.83);
        public static final Translation3d redHubPosition  = new Translation3d(11.92, 4.04, 1.83);

        // Neutral zone (between the BUMPS) - X bounds only, full field width
        // When robot X is inside this range, switch to ferry-aiming at a lateral midpoint
        public static final double neutralZoneMinX = 4.59;   // Blue BUMPS center
        public static final double neutralZoneMaxX = 11.95;  // Red  BUMPS center
        public static final double fieldWidth       = 8.07;  // Full field Y (guardrail to guardrail)

        // Ferry target points: midpoint between HUB and nearest guardrail (y=0 or y=8.07)
        // Blue Alliance: HUB at x=4.63
        public static final Translation3d blueFerryPointRight = new Translation3d(4.63,  2.02, 1.83); // toward y=0
        public static final Translation3d blueFerryPointLeft  = new Translation3d(4.63,  6.05, 1.83); // toward y=8.07
        // Red Alliance: HUB at x=11.92
        public static final Translation3d redFerryPointRight  = new Translation3d(11.92, 2.02, 1.83); // toward y=0
        public static final Translation3d redFerryPointLeft   = new Translation3d(11.92, 6.05, 1.83); // toward y=8.07
    }
    
    public class constDrivetrain {
        public static final int joystickPort = 0;
        public static final double maxAngularRate = 1.5 * Math.PI; // rad/s (~270 deg/s)
        public static final double deadbandPercent = 0.1;

        // Advanced Drive Control Constants
        public static final double deadband = 0.1;
        public static final double halfSpeedFactor = 0.35;
        public static final double rotationActiveTimeout = 0.1; // seconds
        public static final double rotationActiveThresholdDegrees = 3.0; // degrees
        public static final double inputCurve = 3.0; // Input exponent (1.0 = linear, 2.0 = squared, etc.)

        // Speed Control Constants
        public static final double maxSpeed = Units.feetToMeters(14.5); 
        public static final double speedModifier = 0.2; 

        // Dimensions
        public static final double chassisWidth = Units.inchesToMeters(27.0);
        public static final double chassisLength = Units.inchesToMeters(27.0);
    }
    public class constVision {
        
        // Basic filtering thresholds - RELAXED for better detection
        public static final double maxAmbiguity = 0.7; // Was 0.4 - too strict! Allow more uncertain single-tag detections
        public static final double maxZError = 1.0; // Was 0.3m - too strict! Allow ±1m Z error (floor-level uncertainty is normal)

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
        public static final String mainCameraName = "MainCamera";
        public static final String leftCameraName = "LeftCamera";
        public static final String rightCameraName = "RightCamera";

        // Main camera: Center front of robot
        // Left camera: Front-left of robot
        // Right camera: Front-right of robot
        // Using simple positions for testing - adjust based on actual robot measurements
        public static final Transform3d mainCameraOffset = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.28), Units.inchesToMeters(12.309), Units.inchesToMeters(16.158)),
                new Rotation3d(0, Math.toRadians(-20), 0)); // Look forward-center

        public static final Transform3d leftCameraOffset = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45))); // Look forward-left

        public static final Transform3d rightCameraOffset = new Transform3d(
                new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-12), Units.inchesToMeters(9.3)),
                new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45))); // Look forward-right


        // Standard deviation multipliers for each camera (lower = more trusted)
        // Index 0 = Main, 1 = Left, 2 = Right
        public static final double[] cameraStdDevFactors = new double[] {
                1.0, // Main camera (most trusted - center position, best view)
                1.5, // Left camera (slightly less trusted - side angle)
                1.5  // Right camera (slightly less trusted - side angle)
        };

        // Multipliers to apply for MegaTag 2 observations
        public static final double linearStdDevMegatag2Factor = 0.5;
        public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

        // Camera simulation properties
        public static final int cameraFPS = 30;
        public static final int cameraResolutionWidth = 640;
        public static final int cameraResolutionHeight = 480;
        public static final double cameraFOVDegrees = 90.0;
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

        

        public static double kP = 2.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
    }

    public class constTurret {
        public static final int turretMotorId = 33; // TODO: set correct motor ID (was 33)
        
        // Turret angle limits (degrees, robot-relative)
        // 0° = forward, positive = CCW when viewed from above
        public static final double maxTurretAngleDegrees =  180.0;  // CCW limit
        public static final double minTurretAngleDegrees = -180.0;  // CW limit

        // Measured motor positions at known angles (from physical testing)
        //  -0.5 rot  =  180° (CCW hard stop)
        // -21.1 rot  =    0° (forward / home)
        // -42.2 rot  = -180° (CW hard stop)
        public static final double homeMotorPos      = -21.1; // motor rotations at 0° (forward)
        public static final double maxTurretMotorPos =  -0.5; // motor rotations at +180° (software forward limit)
        public static final double minTurretMotorPos = -42.2; // motor rotations at -180° (software reverse limit)

        // Conversion: motor rotations per degree
        // Range = -0.5 - (-42.2) = 41.7 rot over 360°  =>  0.11583... rot/deg
        // Note: motor position DECREASES as angle DECREASES (same direction)
        public static final double angleToPosFactor =
            (maxTurretMotorPos - minTurretMotorPos) /
            (maxTurretAngleDegrees - minTurretAngleDegrees); // positive value

        public static final double lookaheadTimeMs = 200;

        // PID gains
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public class constIntake {
        public static final int intakeMotorId = 41; // 41
        public static final int pivotMotorId = 42; // 42

        public static final double dutyCycle = 0.7; // Duty cycle (0.0 to 1.0)

        public static final double pivotKp = 1.0;
        public static final double pivotKi = 0.0;   
        public static final double pivotKd = 0.0;

        public static final double deployedPos = 30.0; // Motor rotations for deployed position
        public static final double retractedPos = 1.0; // Motor rotations for retracted position
        public static final double pumpPos = 22.0; // Motor rotations for pumped position
        
        // Pump timing
        public static final double pumpDelaySeconds = 0.5; // Time to wait between deployed and pumped positions
    }

    public class constIndexer {
        public static final int indexerMotorId = 51; // 5x = Indexer system
        
        public static final double dutyCycle = 1.0; // Duty cycle (0.0 to 1.0)
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public class constKicker {
        public static final int kickerMotorId = 61; // 6x = Kicker system
        
        public static final double dutyCycle = 1.0; // Duty cycle (0.0 to 1.0)
        
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public class constFlywheel {
        public static final int flywheelMotorId = 32; // 3x = Shooter system
        public static final double maxFlywheelRPM = 5500;
        public static final double minFlywheelRPM = 0;

        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.555; 
        public static final double kV = 0.127; 
        public static final double kA = 0.1; 

    }

    public class constBallisticSolver {
        // Shooter physical constants
        public static final double shooterHeightMeters = Units.inchesToMeters(41.25); // Height of shooter off ground (meters)
        public static final double gravity = 9.806; // m/s^2
        
        // Flywheel configuration
        public static final double flywheelDiameterMeters = Units.inchesToMeters(4); // 4 inches
        public static final double exitVelocityFactor = 0.85; // Tune this: ball exit speed / wheel surface speed
        public static final double gearRatioMotorToWheel = 24.0 / 18.0; // Belt ratio: 24T motor : 18T flywheel = 1.333
        public static final double speedMod = 1.0; // Fine-tune multiplier applied after solver

        // RPM sweep constraints
        public static final double minMotorRPM = 1000.0; // Don't sweep below this - ball won't reach target
        public static final double maxMotorRPM = constFlywheel.maxFlywheelRPM; // Motor RPM ceiling (6000)
        public static final double rpmStep = 50.0; // 50 RPM steps = ~100 iterations max, precise enough

        // Impact angle targeting (degrees, negative = descending)
        // Goal: ball lands from above, descending at ~60 deg from horizontal
        public static final double desiredImpactAngleDeg = -60.0; // Ideal descent angle
        public static final double impactBandMinDeg = -70.0;      // Steepest acceptable (more negative)
        public static final double impactBandMaxDeg = -45.0;      // Shallowest acceptable (less negative)
    }
}