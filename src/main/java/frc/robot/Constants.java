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
        // Yaw (azimuth) motor ID
        public static final int turretYawMotorID = 10;
        
        // Pitch (elevation) motor ID
        public static final int turretPitchMotorID = 11;

        // Conversion factor: motor rotations per degree of turret rotation
        // Adjust based on your gearing (e.g., if 10:1 gearing, then 10/360 = 0.0278)
        public static final double rotationsPerDegree = 1.0 / 360.0; // 1 motor rotation = 360 degrees (adjust for gearing)

        // ===== YAW (AZIMUTH) CONTROL =====
        // Motion Magic parameters for yaw
        public static final double yawMotionVelocity = 360.0; // degrees per second cruise velocity
        public static final double yawMotionAcceleration = 720.0; // degrees per second^2 acceleration
        public static final double yawExpoKA = 0.0; // Exponential acceleration gain
        public static final double yawExpoKV = 0.0; // Exponential velocity gain

        // Feedforward gains for yaw (Phoenix 6 MotionMagic)
        public static final double yawKG = 0.0;  // Gravity feedforward (volts) - not needed for horizontal turret
        public static final double yawKS = 0.1;  // Static friction feedforward (volts)
        public static final double yawKV = 0.12; // Velocity feedforward (volts per rotation/sec)
        public static final double yawKA = 0.01; // Acceleration feedforward (volts per rotation/sec^2)

        // PID gains for turret yaw control (Phoenix 6 MotionMagicVoltage, Slot 0)
        public static final double yawKP = 30.0;  // Proportional gain
        public static final double yawKI = 0.0;   // Integral gain
        public static final double yawKD = 0.0;   // Derivative gain

        // Software limits for yaw (in degrees)
        public static final double minYawDegrees = -180.0;
        public static final double maxYawDegrees = 180.0;
        
        // ===== PITCH (ELEVATION) CONTROL =====
        // Motion Magic parameters for pitch
        public static final double pitchMotionVelocity = 180.0; // degrees per second cruise velocity
        public static final double pitchMotionAcceleration = 360.0; // degrees per second^2 acceleration
        public static final double pitchExpoKA = 0.0; // Exponential acceleration gain
        public static final double pitchExpoKV = 0.0; // Exponential velocity gain

        // Feedforward gains for pitch (Phoenix 6 MotionMagic)
        public static final double pitchKG = 0.5;  // Gravity feedforward (volts) - IMPORTANT for elevation!
        public static final double pitchKS = 0.15; // Static friction feedforward (volts)
        public static final double pitchKV = 0.12; // Velocity feedforward (volts per rotation/sec)
        public static final double pitchKA = 0.01; // Acceleration feedforward (volts per rotation/sec^2)

        // PID gains for turret pitch control (Phoenix 6 MotionMagicVoltage, Slot 0)
        public static final double pitchKP = 35.0;  // Proportional gain
        public static final double pitchKI = 0.0;   // Integral gain
        public static final double pitchKD = 0.5;   // Derivative gain for damping

        // Software limits for pitch (in degrees) - launch angle range
        public static final double minPitchDegrees = 0.0;   // Horizontal
        public static final double maxPitchDegrees = 90.0;  // Vertical
        
        // ===== SHOOTER PHYSICS CONSTANTS =====
        // These are used for calculating launch angles based on projectile motion
        public static final double shooterVelocity = 8.6;      // Launch velocity in m/s
        public static final double shooterHeight = 0.135;      // Height of shooter off ground in meters
        public static final double targetHeight = 1.0;         // Height of target (AprilTag center) in meters
        
        // ===== LOOK-AHEAD / MOTION COMPENSATION =====
        // These constants account for robot motion and system latency during aiming
        
        // Code loop period in seconds (20ms = 0.02s for standard FRC loop)
        public static final double loopPeriodSeconds = 0.02;
        
        // Total system latency in seconds (vision processing + motor response + mechanical delay)
        // This is the time from when we calculate aim to when the projectile actually launches
        public static final double systemLatencySeconds = 0.1;  // 100ms total system latency
        
        // Look-ahead time for prediction (how far ahead to predict robot position)
        // This should account for: latency + time for turret to reach target + launch decision time
        public static final double lookAheadTimeSeconds = 0.15;  // 150ms look-ahead
        
        // Enable/disable motion compensation (useful for testing)
        public static final boolean enableMotionCompensation = true;
        
        // Minimum velocity threshold to apply compensation (m/s)
        // Below this, we don't apply compensation to avoid jitter when stationary
        public static final double minVelocityThreshold = 0.05;  // 5 cm/s
        
        // Minimum angular velocity threshold to apply rotation compensation (rad/s)
        public static final double minAngularVelocityThreshold = 0.05;  // ~3 deg/s
    }
}
