package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {


   

    public class constAutoAim {
        // Testing mode: aim turret directly at closest AprilTag (no odometry logic)
        // Set to false for competition to use odometry + neutral zone ferry logic
        public static final boolean useTagYawForTesting = false;

        // Testing mode for elevation: aim hood/flywheel at AprilTag center height
        // Set to false for competition to use alliance HUB height
        public static final boolean useTagCenterForTesting = false;

        // HUB positions on the field (x, y, z) in meters
        public static final Translation3d blueHubPosition = new Translation3d(Units.inchesToMeters(160-30),  4.04, Units.inchesToMeters(79));
        public static final Translation3d redHubPosition  = new Translation3d(Units.inchesToMeters(651.2-179.1+12), 4.04, Units.inchesToMeters(79));

        // Neutral zone (between the BUMPS) - X bounds only, full field width
        // When robot X is inside this range, switch to ferry-aiming at a lateral midpoint
        public static final double neutralZoneMinX = 4.59;   // Blue BUMPS center
        public static final double neutralZoneMaxX = 11.95;  // Red  BUMPS center
        public static final double fieldWidth       = 8.07;  // Full field Y (guardrail to guardrail)

        // Ferry target points: midpoint between HUB and nearest guardrail (y=0 or y=8.07)
        // Blue Alliance: HUB at x=4.63
        public static final Translation3d blueFerryPointRight = new Translation3d(4.63,  1, 2); // toward y=0
        public static final Translation3d blueFerryPointLeft  = new Translation3d(4.63,  7, 2); // toward y=8.07
        // Red Alliance: HUB at x=11.92
        public static final Translation3d redFerryPointRight  = new Translation3d(11.92, 1, 2); // toward y=0
        public static final Translation3d redFerryPointLeft   = new Translation3d(11.92, 7, 2); // toward y=8.07
    }
    
    public class constDrivetrain {
        public static final int joystickPort = 0;
        public static final double maxAngularRate = 1.0 * Math.PI; // rad/s (~270 deg/s)

        // Advanced Drive Control Constants
        public static final double deadband = 0.1;
        public static final double halfSpeedFactor = 0.35;
        public static final double rotationActiveTimeout = 0.1; // seconds
        public static final double rotationActiveThresholdDegrees = 3.0; // degrees
        public static final double inputCurve = 3.0; // Input exponent (1.0 = linear, 2.0 = squared, etc.)

        // Speed Control Constants
        public static final double maxSpeed = Units.feetToMeters(14.5); 
        public static final double speedModifier = 1.0; 

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
        // Angular stddev is in RADIANS for WPILib's pose estimator.
        // Lowered so vision heading corrections actually get fused into the gyro.
        public static final double linearStdDevBaseline = 0.01; // Meters
        public static final double angularStdDevBaseline = 0.1; // Radians (~5.7°) — was 5.0 (degrees, too large)

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
        // Camera was mounted at the front in code; move to back by negating X.
        // Also rotate yaw by 180° so the camera faces the same field direction
        // when mounted at the rear.
        new Translation3d(Units.inchesToMeters(-12.28), Units.inchesToMeters(12.309), Units.inchesToMeters(16.158)),
        new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45) + Math.PI)); // Look rear-center

        public static final Transform3d leftCameraOffset = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(12), Units.inchesToMeters(9.3)),
        new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45) + Math.PI)); // Look rear-left

        public static final Transform3d rightCameraOffset = new Transform3d(
        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12), Units.inchesToMeters(9.3)),
        new Rotation3d(0, Math.toRadians(-20), Math.toRadians(45) + Math.PI)); // Look rear-right


        // Standard deviation multipliers for each camera (lower = more trusted)
        // Index 0 = Main, 1 = Left, 2 = Right
        public static final double[] cameraStdDevFactors = new double[] {
                1.0, // Main camera (most trusted - center position, best view)
                1.5, // Left camera (slightly less trusted - side angle)
                1.5  // Right camera (slightly less trusted - side angle)
        };

        // Multipliers to apply for MegaTag 2 observations
        // Angular factor is NOT infinity so that multi-tag heading corrections are fused.
        public static final double linearStdDevMegatag2Factor = 0.5;
        public static final double angularStdDevMegatag2Factor = 1.0; // was POSITIVE_INFINITY — now fuses heading

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

        // Non-physical software operating ceiling (separate from physical soft limit).
        // Tune this down to keep hood travel below full mechanical range.
        public static final double softMaxHoodMotorPos = maxHoodMotorPos;

        /**
         * Angle convention switch:
         * true  -> hood angle is complementary to launch angle (launch = 90 - hood),
         *         e.g. hood 20° means launch 70°.
         * false -> hood angle already equals launch angle from horizontal.
         */
        public static final boolean hoodAngleIsComplementOfLaunch = true;

        /** Convert mechanism hood angle (deg) to ballistic launch angle (deg above horizontal). */
        public static double hoodAngleToLaunchAngleDeg(double hoodAngleDeg) {
            return hoodAngleIsComplementOfLaunch ? (90.0 - hoodAngleDeg) : hoodAngleDeg;
        }

        /** Convert ballistic launch angle (deg above horizontal) to mechanism hood angle (deg). */
        public static double launchAngleToHoodAngleDeg(double launchAngleDeg) {
            return hoodAngleIsComplementOfLaunch ? (90.0 - launchAngleDeg) : launchAngleDeg;
        }

        // Solver launch-angle limits derived from physical hood limits.
        public static final double minLaunchAngleDegrees = Math.min(
                hoodAngleToLaunchAngleDeg(minHoodAngleDegrees),
                hoodAngleToLaunchAngleDeg(maxHoodAngleDegrees));
        public static final double maxLaunchAngleDegrees = Math.max(
                hoodAngleToLaunchAngleDeg(minHoodAngleDegrees),
                hoodAngleToLaunchAngleDeg(maxHoodAngleDegrees));

        public static final double angleToPosFactor =
            (maxHoodMotorPos - minHoodMotorPos) /
            (maxHoodAngleDegrees - minHoodAngleDegrees);

        // Derived commanded ceiling angle from soft motor-position cap.
        public static final double softMaxHoodAngleDegrees =
            minHoodAngleDegrees +
            (softMaxHoodMotorPos - minHoodMotorPos) / angleToPosFactor;

    // Solver launch-angle limits derived from OPERATING hood limits (min to soft-max).
    // Use these when you want ballistic solutions to obey the non-physical soft cap.
    public static final double minLaunchAngleSoftDegrees = Math.min(
        hoodAngleToLaunchAngleDeg(minHoodAngleDegrees),
        hoodAngleToLaunchAngleDeg(softMaxHoodAngleDegrees));
    public static final double maxLaunchAngleSoftDegrees = Math.max(
        hoodAngleToLaunchAngleDeg(minHoodAngleDegrees),
        hoodAngleToLaunchAngleDeg(softMaxHoodAngleDegrees));

        

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

        // Conversion: motor rotations per degree.
        // Derived from measured endpoints through the home position:
        //   +180° → -0.5 rot   ⟹  factor = (-0.5 - (-21.1)) / 180  = +0.11444 rot/deg
        //   -180° → -42.2 rot  ⟹  factor = (-42.2 - (-21.1)) / -180 = +0.11722 rot/deg
        // Use the CCW half (0° → +180°) as the reference since that is the home side.
        // Positive value: motor position increases (less negative) as angle increases (CCW).
        public static final double angleToPosFactor =
            (maxTurretMotorPos - homeMotorPos) / maxTurretAngleDegrees; // +0.11444 rot/deg

        public static final double lookaheadTimeMs = 200;

        // Offset (degrees) added to the auto-aim angle to correct for the turret's
        // physical zero not matching the robot's gyro zero.
        // The motor was zeroed at the 180° position (-0.5 rot), so the turret
        // forward (0°) corresponds to -21.1 motor rotations.
        // Tune this if the turret points in the wrong direction:
        //   pointing CW of target  → increase this value
        //   pointing CCW of target → decrease this value
    
        public static final double turretAngleOffsetDegrees = 90+15;
     
        

        // PID gains
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double turretCenterX = 8;
        public static final double turretCenterY = 8;
    }

    public class constIntake {
        public static final int intakeMotorId = 41; // 41
        public static final int pivotMotorId = 42; // 42

        public static final double dutyCycle = 0.75; // Duty cycle (0.0 to 1.0)

        public static final double pivotKp = 1.0;
        public static final double pivotKi = 0.0;   
        public static final double pivotKd = 0.0;

        public static final double deployedPos = 32.0; // Motor rotations for deployed position
        public static final double deployedTolerance = 0.2; // Rotations of acceptable error to consider "deployed"
        public static final double retractedPos = 1.0; // Motor rotations for retracted position
        public static final double pumpPos = 18.0; // Motor rotations for pumped position

        // Pump timing
        public static final double pumpDelaySeconds = 0.5; // Time to wait between deployed and pumped positions
    }

    public class constIndexer {
        public static final int indexerMotorId = 51; // 5x = Indexer system
        
        public static final double dutyCycle = 1.0; // Duty cycle (0.0 to 1.0)
        public static final double idleDutyCycle = 0; // Minimum duty cycle to hold balls in place without jamming
        public static final double reverseDutyCycle = -0.5; // Duty cycle for reverse (unjam)
    }

    public class constKicker {
        public static final int kickerMotorId = 61; // 6x = Kicker system
        
        public static final double dutyCycle = 1.0; // Duty cycle (0.0 to 1.0)
        public static final double reverseDutyCycle = -0.5; // Duty cycle for reverse (unjam)
        
     
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
        public static final double shooterHeightMeters = Units.inchesToMeters(17.0625); // Height of shooter off ground (meters)
        public static final double gravity = 9.806; // m/s^2
        
        // Flywheel configuration
        public static final double flywheelDiameterMeters = Units.inchesToMeters(4); // 4 inches
        public static final double exitVelocityFactor = 1.0; // Tune this: ball exit speed / wheel surface speed
        public static final double gearRatioMotorToWheel = 24.0 / 18.0; // Belt ratio: 24T motor : 18T flywheel = 1.333
        public static final double speedMod = 0.825; // Fine-tune multiplier applied after solver

    // Empirical RPM compensation term (tune on field):
    // Flight-time-based correction for drag losses (longer flight => more RPM).
    public static final double rpmPerSecondOfFlightCompensation = 0.0;

        // RPM sweep constraints
        public static final double minMotorRPM = 1000.0;
        public static final double maxMotorRPM = constFlywheel.maxFlywheelRPM;
        public static final double rpmStep = 50.0;

        // Obstacle clearance gate — ball must clear the 6ft rim before entering the goal.
        // rimOffsetMeters: how far the rim is in FRONT of the goal center (the rim is before the target).
        // clearanceXMeters is computed dynamically each cycle as (range - rimOffsetMeters).
        public static final double rimOffsetMeters  = Units.inchesToMeters(4); // Rim is ~12 inches in front of goal center
        public static final double clearanceZMeters = Units.inchesToMeters(72); // 6 ft rim height

        // Impact angle targeting (degrees, negative = descending)
        // HIGH launch angle (lofted arc) clears the 6ft rim and drops steeply into the top of the goal.
        // The solver now sweeps ALL RPMs and picks whichever solution is closest to desiredImpactAngleDeg.
        public static final double desiredImpactAngleDeg = -80.0; // Ideal steep descent into goal top
        public static final double impactBandMinDeg = -90.0;      // Steepest acceptable
        public static final double impactBandMaxDeg = -70.0;      // Shallowest acceptable
    }
}