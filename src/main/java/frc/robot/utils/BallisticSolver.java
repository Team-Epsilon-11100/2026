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

    // Launch angle limits (degrees above horizontal), derived from operating
    // hood range (min to soft max) so solver obeys non-physical soft cap.
    public double minAngleDeg              = constHood.minLaunchAngleSoftDegrees;
    public double maxAngleDeg              = constHood.maxLaunchAngleSoftDegrees;

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
        // clearanceXMeters is set dynamically each cycle based on current range — do not use a static default.
        public double clearanceZMeters         = constBallisticSolver.clearanceZMeters;
        public double clearanceXMeters         = 0.0; // Set per-cycle in AutoElevationCommand
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

    /**
     * Simpler angle-first solver:
     *  - Sweep hood angle high -> low (prefer lofted arcs)
     *  - For each angle, compute required exit speed analytically
     *  - Check RPM limits, optional clearance gate, and impact-angle validity
     *  - Return first strict in-band hit; otherwise keep best relaxed hit
     */
    public static Solution solveLowestRpmPreferImpact(
            double xMeters, double yMeters, double goalZMeters, Config cfg) {
        return solveByAngleSweep(xMeters, yMeters, goalZMeters, cfg, cfg.maxAngleDeg, cfg.minAngleDeg, -1.0, "angle-sweep");
    }

    /**
     * Keep compatibility with existing call sites.
     * Strategy:
     *  1) Try fixed min hood angle first (flat shot requirement)
     *  2) If not feasible, fall back to simple high->low angle sweep.
     */
    public static Solution solveHoodMinThenRpmSweep(
            double xMeters, double yMeters, double goalZMeters, Config cfg) {

        double range  = Math.hypot(xMeters, yMeters);
        double deltaZ = goalZMeters - cfg.shooterZMeters;
        if (range <= 0.0) {
            return Solution.invalid("range is zero");
        }

        // First try the launch angle that corresponds to PHYSICAL hood minimum.
        double launchAtHoodMin = constHood.hoodAngleToLaunchAngleDeg(constHood.minHoodAngleDegrees);
        Solution fixedMin = solveForFixedAngle(range, deltaZ, launchAtHoodMin, cfg, "min-hood");
        if (fixedMin.valid()) {
            return fixedMin;
        }

        return solveByAngleSweep(xMeters, yMeters, goalZMeters, cfg, cfg.maxAngleDeg, cfg.minAngleDeg, -1.0, "fallback-sweep");
    }

    private static Solution solveByAngleSweep(
            double xMeters,
            double yMeters,
            double goalZMeters,
            Config cfg,
            double startAngleDeg,
            double endAngleDeg,
            double stepDeg,
            String reasonTag) {

        double range  = Math.hypot(xMeters, yMeters);
        double deltaZ = goalZMeters - cfg.shooterZMeters;
        if (range <= 0.0) {
            return Solution.invalid("range is zero");
        }

        if (stepDeg == 0.0) {
            return Solution.invalid("step cannot be zero");
        }

        Solution bestRelaxed = null;

        for (double angle = startAngleDeg;
             (stepDeg > 0 ? angle <= endAngleDeg + 1e-9 : angle >= endAngleDeg - 1e-9);
             angle += stepDeg) {

            if (angle < cfg.minAngleDeg || angle > cfg.maxAngleDeg) continue;

            Solution s = solveForFixedAngle(range, deltaZ, angle, cfg, reasonTag);
            if (!s.valid()) continue;

            boolean inBand = s.impactAngleDeg() >= cfg.impactBandMinDeg
                    && s.impactAngleDeg() <= cfg.impactBandMaxDeg;

            // Prefer first strict solution during angle sweep (simple & deterministic)
            if (inBand) {
                return s;
            }

            // Keep best relaxed solution by proximity to desired impact
            if (bestRelaxed == null
                    || impactScore(s.impactAngleDeg(), cfg) > impactScore(bestRelaxed.impactAngleDeg(), cfg)) {
                bestRelaxed = s;
            }
        }

        if (bestRelaxed != null) return bestRelaxed;
        return Solution.invalid("no trajectory found in angle range");
    }

    private static Solution solveForFixedAngle(
            double range,
            double deltaZ,
            double angleDeg,
            Config cfg,
            String reasonTag) {

        double angleRad = Math.toRadians(angleDeg);
        double cos = Math.cos(angleRad);
        double sin = Math.sin(angleRad);
        double cos2 = cos * cos;

        // denominator from projectile equation rearrangement
        double denominator = range * Math.tan(angleRad) - deltaZ;
        if (denominator <= 0 || cos2 <= 1e-12) {
            return Solution.invalid("unreachable at angle");
        }

        // v^2 = g*R^2 / (2*cos^2(theta)*(R*tan(theta) - deltaZ))
        double v2 = cfg.g * range * range / (2.0 * cos2 * denominator);
        if (v2 <= 0) {
            return Solution.invalid("invalid speed");
        }

        double exitSpeed = Math.sqrt(v2);
        double wheelRpm = exitSpeedToWheelRpm(exitSpeed, cfg);
        double idealMotorRpm = wheelRpmToMotorRpm(wheelRpm, cfg);

        double vx = exitSpeed * cos;
        double vy = exitSpeed * sin;
        if (vx <= 1e-9) {
            return Solution.invalid("vx too small");
        }

        double tFlight = range / vx;

    // Empirical compensation:
    //  - time-based term for drag (longer flight needs more exit energy)
    double compensatedMotorRpm = idealMotorRpm
        * (1.0 + constBallisticSolver.rpmPerSecondOfFlightCompensation * tFlight);

    if (compensatedMotorRpm < cfg.minMotorRpm || compensatedMotorRpm > cfg.maxMotorRpm) {
        return Solution.invalid("rpm out of range");
    }

        // Optional obstacle clearance
        if (cfg.clearanceXMeters > 0 && cfg.clearanceZMeters > 0) {
            double tClear = cfg.clearanceXMeters / vx;
            double zAtClearance = cfg.shooterZMeters
                    + vy * tClear
                    - 0.5 * cfg.g * tClear * tClear;
            if (zAtClearance < cfg.clearanceZMeters) {
                return Solution.invalid("fails clearance");
            }
        }

        // Impact angle at target (negative = descending)
        double vyFinal = vy - cfg.g * tFlight;
        double impactAngleDeg = Math.toDegrees(Math.atan2(vyFinal, vx));
        if (cfg.requireDescendingAtTarget && impactAngleDeg >= 0) {
            return Solution.invalid("not descending");
        }

        return new Solution(true, angleDeg, impactAngleDeg, exitSpeed, wheelRpm, compensatedMotorRpm, tFlight, reasonTag);
    }

    // -----------------------------------------------------------------------
    // Physics helpers
    // -----------------------------------------------------------------------

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