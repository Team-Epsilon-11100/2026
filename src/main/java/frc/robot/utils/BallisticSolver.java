package frc.robot.utils;

import frc.robot.Constants.constBallisticSolver;
import frc.robot.Constants.constHood;

/**
 * BallisticSolver (angle-first, high-arc preferred)
 *
 * Selection order:
 *  1) Sweep launch angle from MAX -> MIN.
 *  2) Compute required wheel speed for each angle.
 *  3) Enforce wheel/motor speed limits.
 *  4) Enforce minimum impact angle.
 *  5) Return first valid (highest-angle) solution.
 */
public class BallisticSolver {

    // -----------------------------------------------------------------------
    // Config
    // -----------------------------------------------------------------------
    public static class Config {
        // Physics / geometry
        public double shooterZMeters           = constBallisticSolver.shooterHeightMeters;
        public double g                        = constBallisticSolver.gravity;

    // Launch angle sweep (ballistic launch angle, deg above horizontal)
    public double minAngleDeg              = constHood.minLaunchAngleSoftDegrees;
    public double maxAngleDeg              = constHood.maxLaunchAngleSoftDegrees;
        public double angleStepDeg             = 1.0;
        public double minImpactAngleDeg        = 30.0;

        // Flywheel model
        public double flywheelDiameterMeters   = constBallisticSolver.flywheelDiameterMeters;
        public double exitVelocityFactor       = constBallisticSolver.exitVelocityFactor;
        public double gearRatioMotorToWheel    = constBallisticSolver.gearRatioMotorToWheel;

        // Motor limits
        public double minMotorRpm              = constBallisticSolver.minMotorRPM;
        public double maxMotorRpm              = constBallisticSolver.maxMotorRPM;

    // Linear RPM compensation: add (rpmCompPerSecond * flightTimeSec)
    public double rpmCompPerSecond         = constBallisticSolver.rpmPerSecondOfFlightCompensation;
    }

    // -----------------------------------------------------------------------
    // Solution record
    // -----------------------------------------------------------------------
    public record Solution(
        boolean valid,
        double  launchAngleDeg,
        double  impactAngleDeg,
        double  exitSpeedMps,
        double  wheelRpm,
        double  motorRpm,
        double  timeSec,
        String  reason
    ) {
        /** Convenience factory for an invalid (no-solution) result. */
        public static Solution invalid(String reason) {
            return new Solution(false, 0, 0, 0, 0, 0, 0, reason);
        }
    }

    // -----------------------------------------------------------------------
    // Primary entry point
    // -----------------------------------------------------------------------
    /**
    * Finds the highest valid launch angle in range, using the required wheel speed
    * at each candidate angle.
     *
     * @param xMeters     horizontal X displacement to goal (metres)
     * @param yMeters     horizontal Y displacement to goal (metres)
     * @param goalZMeters absolute height of target (metres above floor)
     * @param cfg         solver config (create once, reuse every cycle)
     * @return best Solution, or Solution.invalid(...) if none found
     */
    public static Solution solveLowestRpmPreferImpact(
            double xMeters, double yMeters, double goalZMeters, Config cfg) {

        double range = Math.hypot(xMeters, yMeters);
        double deltaZ = goalZMeters - cfg.shooterZMeters;

        if (range <= 0.0) {
            return Solution.invalid("range is zero");
        }

        if (cfg.angleStepDeg <= 0.0) {
            return Solution.invalid("angleStepDeg must be > 0");
        }

        double wheelRadius = cfg.flywheelDiameterMeters / 2.0;
        if (wheelRadius <= 0.0) {
            return Solution.invalid("flywheel diameter must be > 0");
        }

        double minWheelRps = (cfg.minMotorRpm / cfg.gearRatioMotorToWheel) / 60.0;
        double maxWheelRps = (cfg.maxMotorRpm / cfg.gearRatioMotorToWheel) / 60.0;

        for (double angleDeg = cfg.maxAngleDeg; angleDeg >= cfg.minAngleDeg - 1e-9; angleDeg -= cfg.angleStepDeg) {
            double thetaRad = Math.toRadians(angleDeg);
            double cosTheta = Math.cos(thetaRad);
            if (cosTheta <= 1e-9) {
                continue;
            }

            // denominator = x*tan(theta) - y  (must be > 0)
            double denominator = range * Math.tan(thetaRad) - deltaZ;
            if (denominator <= 0.0) {
                continue;
            }

            // K = [1/(2*pi*r*cos(theta)*exitVelocityFactor)] * sqrt(g/2)
            double k = (1.0 / (2.0 * Math.PI * wheelRadius * cosTheta * cfg.exitVelocityFactor))
                    * Math.sqrt(cfg.g / 2.0);
            double termUnderRadical = (range * range) / denominator;
            if (termUnderRadical <= 0.0) {
                continue;
            }

            // Required wheel speed in RPS
            double wheelRps = k * Math.sqrt(termUnderRadical);
            if (wheelRps < minWheelRps || wheelRps > maxWheelRps) {
                continue;
            }

            // Re-derive linear launch speed for impact-angle validation
            double exitSpeed = wheelRpsToExitSpeed(wheelRps, cfg);
            double vSinThetaSq = Math.pow(exitSpeed * Math.sin(thetaRad), 2);
            double twoGY = 2.0 * cfg.g * deltaZ;

            // Must reach target height
            if (vSinThetaSq < twoGY) {
                continue;
            }

            double impactNum = Math.sqrt(vSinThetaSq - twoGY);
            double impactDen = exitSpeed * cosTheta;
            if (impactDen <= 1e-9) {
                continue;
            }

            // Positive magnitude like your script, convert to signed descending angle for telemetry
            double impactAngleMagnitudeDeg = Math.toDegrees(Math.atan(impactNum / impactDen));
            if (impactAngleMagnitudeDeg < cfg.minImpactAngleDeg) {
                continue;
            }

            double baseWheelRpm = wheelRps * 60.0;
            double baseMotorRpm = wheelRpmToMotorRpm(baseWheelRpm, cfg);
            double vx = exitSpeed * cosTheta;
            double tFlight = (vx > 1e-9) ? (range / vx) : 0.0;

            // Linear compensation on top of solved RPM
            // motorRpm = baseMotorRpm + (rpmCompPerSecond * flightTimeSec)
            double compensationRpm = cfg.rpmCompPerSecond * tFlight;
            double motorRpm = baseMotorRpm + compensationRpm;
            if (motorRpm < cfg.minMotorRpm || motorRpm > cfg.maxMotorRpm) {
                continue;
            }
            double wheelRpm = motorRpm / cfg.gearRatioMotorToWheel;

            return new Solution(
                    true,
                    angleDeg,
                    -impactAngleMagnitudeDeg,
                    exitSpeed,
                    wheelRpm,
                    motorRpm,
                    tFlight,
                    "ok"
            );
        }

        return Solution.invalid("no trajectory found in RPM/angle range");
    }

    // -----------------------------------------------------------------------
    // Unit conversions (public for testing / dashboard use)
    // -----------------------------------------------------------------------

    /** Wheel RPS -> ball exit speed (m/s). */
    public static double wheelRpsToExitSpeed(double wheelRps, Config cfg) {
        double wheelSurfaceSpeed = wheelRps * (Math.PI * cfg.flywheelDiameterMeters);
        return wheelSurfaceSpeed * cfg.exitVelocityFactor;
    }

    /** Motor RPM -> ball exit speed (m/s). */
    public static double motorRpmToExitSpeed(double motorRpm, Config cfg) {
        double wheelRpm          = motorRpm / cfg.gearRatioMotorToWheel;
        double wheelSurfaceSpeed = (wheelRpm / 60.0) * (Math.PI * cfg.flywheelDiameterMeters);
        return wheelSurfaceSpeed * cfg.exitVelocityFactor;
    }

    /** Ball exit speed (m/s) -> wheel RPM. */
    public static double exitSpeedToWheelRpm(double exitSpeedMps, Config cfg) {
        double wheelSurfaceSpeed = exitSpeedMps / cfg.exitVelocityFactor;
        return (wheelSurfaceSpeed / (Math.PI * cfg.flywheelDiameterMeters)) * 60.0;
    }

    /** Wheel RPM -> motor RPM. */
    public static double wheelRpmToMotorRpm(double wheelRpm, Config cfg) {
        return wheelRpm * cfg.gearRatioMotorToWheel;
    }
}