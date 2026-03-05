package frc.robot.utils;

import frc.robot.Constants.constBallisticSolver;
import frc.robot.Constants.constHood;

/**
 * BallisticSolver (RPM-first, LOWEST RPM that works, impact-angle prioritized)
 *
 * GUARANTEED selection order:
 *  1) RPM is the OUTER LOOP. We sweep motor RPM low -> high.
 *  2) For each fixed RPM, we compute feasible launch angle(s) via quadratic formula.
 *  3) For that RPM, we choose the angle that BEST matches the desired impact angle.
 *  4) We return the FIRST RPM that yields an acceptable impact-angle solution.
 *
 * Impact-angle notes:
 *  - impactAngleDeg is the velocity angle at the target (negative = descending).
 *  - "60 deg descent" = impactAngleDeg = -60.
 *
 * Physics model: ideal projectile (no drag, no Magnus).
 */
public class BallisticSolver {

    // -----------------------------------------------------------------------
    // Config
    // -----------------------------------------------------------------------
    public static class Config {
        // Physics / geometry
        public double shooterZMeters           = constBallisticSolver.shooterHeightMeters;
        public double g                        = constBallisticSolver.gravity;

        // Hood angle limits
        public double minAngleDeg              = constHood.minHoodAngleDegrees;
        public double maxAngleDeg              = constHood.maxHoodAngleDegrees;

        // Flywheel model
        public double flywheelDiameterMeters   = constBallisticSolver.flywheelDiameterMeters;
        public double exitVelocityFactor       = constBallisticSolver.exitVelocityFactor;
        public double gearRatioMotorToWheel    = constBallisticSolver.gearRatioMotorToWheel;

        // RPM sweep
        public double minMotorRpm              = constBallisticSolver.minMotorRPM;
        public double maxMotorRpm              = constBallisticSolver.maxMotorRPM;
        public double rpmStep                  = constBallisticSolver.rpmStep;

        // Impact-angle targeting
        public boolean requireDescendingAtTarget = true;
        public double desiredImpactAngleDeg    = constBallisticSolver.desiredImpactAngleDeg;
        public double impactBandMinDeg         = constBallisticSolver.impactBandMinDeg;
        public double impactBandMaxDeg         = constBallisticSolver.impactBandMaxDeg;

        // Optional obstacle clearance (set both to 0 to disable)
        public double clearanceZMeters         = 0.0;
        public double clearanceXMeters         = 0.0;
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
     * Finds the LOWEST motor RPM at which a valid launch exists whose impact
     * angle falls inside [impactBandMinDeg, impactBandMaxDeg].
     *
     * Two-pass strategy:
     *   Pass 0 - strict:  impact must be inside the configured band.
     *   Pass 1 - relaxed: accept any descending shot if pass 0 found nothing.
     *
     * @param xMeters     horizontal X displacement to goal (metres)
     * @param yMeters     horizontal Y displacement to goal (metres)
     * @param goalZMeters absolute height of target (metres above floor)
     * @param cfg         solver config (create once, reuse every cycle)
     * @return best Solution, or Solution.invalid(...) if none found
     */
    public static Solution solveLowestRpmPreferImpact(
            double xMeters, double yMeters, double goalZMeters, Config cfg) {

        double range  = Math.hypot(xMeters, yMeters);
        double deltaZ = goalZMeters - cfg.shooterZMeters;

        if (range <= 0.0) {
            return Solution.invalid("range is zero");
        }

        Solution bestRelaxed = null;

        for (int pass = 0; pass <= 1; pass++) {
            boolean strictBand = (pass == 0);

            for (double rpm = cfg.minMotorRpm; rpm <= cfg.maxMotorRpm + 1e-6; rpm += cfg.rpmStep) {
                double exitSpeed = motorRpmToExitSpeed(rpm, cfg);
                if (exitSpeed <= 0) continue;

                double[] angles = launchAnglesForFixedSpeed(range, deltaZ, exitSpeed, cfg.g);
                if (angles == null) continue;

                // Sort ascending: always try the LOWER launch angle first.
                // A flatter (lower) launch angle produces a steeper descent arc at the target,
                // which is what we want. The quadratic gives no ordering guarantee.
                if (angles.length == 2 && angles[0] > angles[1]) {
                    double tmp = angles[0]; angles[0] = angles[1]; angles[1] = tmp;
                }

                for (double angleDeg : angles) {
                    if (angleDeg < cfg.minAngleDeg || angleDeg > cfg.maxAngleDeg) continue;

                    double angleRad = Math.toRadians(angleDeg);
                    double vx       = exitSpeed * Math.cos(angleRad);
                    double vy       = exitSpeed * Math.sin(angleRad);
                    if (vx <= 1e-9) continue; // Near-vertical launch - can't reach horizontal range
                    double tFlight  = range / vx;

                    // Optional clearance gate
                    if (cfg.clearanceXMeters > 0 && cfg.clearanceZMeters > 0) {
                        double tClear       = cfg.clearanceXMeters / vx;
                        double zAtClearance = cfg.shooterZMeters
                                + vy * tClear
                                - 0.5 * cfg.g * tClear * tClear;
                        if (zAtClearance < cfg.clearanceZMeters) continue;
                    }

                    // Impact angle = atan2(vy_final, vx)
                    double vyFinal     = vy - cfg.g * tFlight;
                    double impactAngle = Math.toDegrees(Math.atan2(vyFinal, vx));

                    if (cfg.requireDescendingAtTarget && impactAngle >= 0) continue;

                    if (strictBand) {
                        if (impactAngle < cfg.impactBandMinDeg || impactAngle > cfg.impactBandMaxDeg) continue;
                        // Lowest RPM with in-band impact angle - return immediately
                        double wheelRpm = exitSpeedToWheelRpm(exitSpeed, cfg);
                        double motorRpm = wheelRpmToMotorRpm(wheelRpm, cfg);
                        return new Solution(true, angleDeg, impactAngle,
                                exitSpeed, wheelRpm, motorRpm, tFlight, "ok");
                    } else {
                        // Relaxed pass: track the angle closest to desiredImpactAngleDeg
                        if (bestRelaxed == null
                                || impactScore(impactAngle, cfg) > impactScore(bestRelaxed.impactAngleDeg(), cfg)) {
                            double wheelRpm = exitSpeedToWheelRpm(exitSpeed, cfg);
                            double motorRpm = wheelRpmToMotorRpm(wheelRpm, cfg);
                            bestRelaxed = new Solution(true, angleDeg, impactAngle,
                                    exitSpeed, wheelRpm, motorRpm, tFlight, "relaxed");
                        }
                    }
                }
            }

            if (pass == 0 && bestRelaxed != null) break;
        }

        if (bestRelaxed != null) return bestRelaxed;
        return Solution.invalid("no trajectory found in RPM/angle range");
    }

    // -----------------------------------------------------------------------
    // Physics helpers
    // -----------------------------------------------------------------------

    /**
     * Exact analytical launch angles that hit (range, deltaZ) at fixed speed v0.
     *
     * Projectile quadratic (u = tan(launchAngle)):
     *   A*u^2 + B*u + C = 0
     *   A =  g*R^2 / (2*v0^2)
     *   B = -R
     *   C =  deltaZ + A
     *
     * @return 1 or 2 angles in degrees, or null if discriminant < 0
     */
    private static double[] launchAnglesForFixedSpeed(
            double range, double deltaZ, double v0, double g) {
        double v2   = v0 * v0;
        double A    = g * range * range / (2.0 * v2);
        double B    = -range;
        double C    = deltaZ + A;
        double disc = B * B - 4.0 * A * C;

        if (disc < 0) return null;

        double sqrtDisc = Math.sqrt(disc);
        double u1 = (-B + sqrtDisc) / (2.0 * A);
        double u2 = (-B - sqrtDisc) / (2.0 * A);
        double a1 = Math.toDegrees(Math.atan(u1));
        double a2 = Math.toDegrees(Math.atan(u2));

        if (Math.abs(disc) < 1e-9) return new double[]{a1};
        return new double[]{a1, a2};
    }

    /** Higher score = impact angle is closer to desiredImpactAngleDeg. */
    private static double impactScore(double impactAngleDeg, Config cfg) {
        return -Math.abs(impactAngleDeg - cfg.desiredImpactAngleDeg);
    }

    // -----------------------------------------------------------------------
    // Unit conversions (public for testing / dashboard use)
    // -----------------------------------------------------------------------

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