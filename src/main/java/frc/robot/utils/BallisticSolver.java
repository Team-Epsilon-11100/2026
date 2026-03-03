package frc.robot.utils;

import frc.robot.Constants.constBallisticSolver;
import frc.robot.Constants.constHood;

public class BallisticSolver {

    public static class Config {
        public double shooterZMeters = constBallisticSolver.shooterHeightMeters;
        public double g = constBallisticSolver.gravity;

        public double minAngleDeg = constHood.minHoodAngleDegrees;
        public double maxAngleDeg = constHood.maxHoodAngleDegrees;
        public double angleStepDeg = constBallisticSolver.angleStepDeg;

        public Double minSpeedMps = constBallisticSolver.minSpeedMps;
        public Double maxSpeedMps = constBallisticSolver.maxSpeedMps;

        public Double preferSpeedDeltaMps = constBallisticSolver.preferredSpeedDeltaMps;

        // --- Flywheel / drivetrain conversion ---
        public double flywheelDiameterMeters = constBallisticSolver.flywheelDiameterMeters;
        /**
         * k = (ball exit speed) / (wheel surface speed)
         * so wheelSurfaceSpeed = exitSpeed / k
         * Start with ~0.85 and tune from real shots.
         */
        public double exitVelocityFactor = constBallisticSolver.exitVelocityFactor;

        /**
         * gearRatioMotorToWheel = motorRPM / wheelRPM
         * Example: 2:1 reduction (motor spins 2x wheel) => 2.0
         * Example: 1:2 overdrive (motor spins half of wheel) => 0.5
         */
        public double gearRatioMotorToWheel = constBallisticSolver.gearRatioMotorToWheel;
    }

    /**
     * Ballistic solution result with all calculated outputs.
     * Use .valid() to check if solution exists, then access other fields.
     * 
     * Example:
     *   Solution s = BallisticSolver.solve(...);
     *   if (s.valid()) {
     *       double angle = s.angleDeg();
     *       double rpm = s.motorRpm();
     *   }
     */
    public record Solution(
        boolean valid,
        double angleDeg,
        double exitSpeedMps,
        double wheelRpm,
        double motorRpm,
        double timeSec,
        String reason
    ) {
        /**
         * Create a valid solution with all calculated values.
         */
        public static Solution ok(double angleDeg, double exitSpeedMps, double wheelRpm, double motorRpm, double timeSec) {
            return new Solution(true, angleDeg, exitSpeedMps, wheelRpm, motorRpm, timeSec, "OK");
        }

        /**
         * Create an invalid solution with a reason string.
         */
        public static Solution invalid(String reason) {
            return new Solution(false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, reason);
        }

        /**
         * Check if this is a valid solution (alternative to checking .valid() directly).
         */
        public boolean isValid() {
            return valid;
        }

        /**
         * Get angle in radians (converted from degrees).
         */
        public double angleRad() {
            return Math.toRadians(angleDeg);
        }

        /**
         * Get a human-readable description of the solution.
         */
        @Override
        public String toString() {
            if (!valid) return "No solution: " + reason;
            return String.format(
                    "angle=%.2f deg, exitV=%.2f m/s, wheel=%.0f rpm, motor=%.0f rpm, time=%.3f s",
                    angleDeg, exitSpeedMps, wheelRpm, motorRpm, timeSec
            );
        }
    }

    /** Convert desired BALL exit speed to wheel RPM for a given flywheel diameter & k factor. */
    public static double exitSpeedToWheelRpm(double exitSpeedMps, Config cfg) {
        double r = cfg.flywheelDiameterMeters / 2.0;
        double k = cfg.exitVelocityFactor;

        if (!(r > 0.0) || !(k > 0.0)) return Double.NaN;

        // wheel surface speed needed:
        double wheelSurfaceSpeed = exitSpeedMps / k;

        // wheelRPM = (v / (2πr)) * 60
        return (wheelSurfaceSpeed / (2.0 * Math.PI * r)) * 60.0;
    }

    /** Convert wheel RPM to motor RPM using motorRPM/wheelRPM ratio. */
    public static double wheelRpmToMotorRpm(double wheelRpm, Config cfg) {
        return wheelRpm * cfg.gearRatioMotorToWheel;
    }

    /** Convert motor RPM to exit speed (m/s) - inverse of the above conversions. */
    public static double motorRpmToExitSpeed(double motorRpm, Config cfg) {
        // motorRPM → wheelRPM
        double wheelRpm = motorRpm / cfg.gearRatioMotorToWheel;
        
        // wheelRPM → wheel surface speed (m/s)
        double r = cfg.flywheelDiameterMeters / 2.0;
        double wheelSurfaceSpeed = (wheelRpm / 60.0) * (2.0 * Math.PI * r);
        
        // wheel surface speed → exit speed
        double exitSpeed = wheelSurfaceSpeed * cfg.exitVelocityFactor;
        
        return exitSpeed;
    }

    public static double requiredSpeedForAngle(double dMeters, double goalZMeters, double angleDeg, Config cfg) {
        double d = dMeters;
        if (d <= 1e-9) return Double.NaN;

        double dz = goalZMeters - cfg.shooterZMeters;
        double a = Math.toRadians(angleDeg);

        double cosA = Math.cos(a);
        double tanA = Math.tan(a);

        double denom = 2.0 * cosA * cosA * (d * tanA - dz);
        if (denom <= 0.0) return Double.NaN;

        double v2 = cfg.g * d * d / denom;
        if (!(v2 > 0.0) || !Double.isFinite(v2)) return Double.NaN;

        return Math.sqrt(v2);
    }

    public static Solution solvePreferConstantSpeed(
            double xMeters,
            double yMeters,
            double goalZMeters,
            double vRefMps,
            Config cfg
    ) {
        double d = Math.hypot(xMeters, yMeters);
        if (d <= 1e-6) return Solution.invalid("Horizontal distance ~0; expected non-zero X/Y distance.");
        if (!Double.isFinite(vRefMps) || vRefMps <= 0.0) return Solution.invalid("vRefMps must be positive and finite.");

        boolean useBand = (cfg.preferSpeedDeltaMps != null && cfg.preferSpeedDeltaMps > 0.0);
        double band = useBand ? cfg.preferSpeedDeltaMps : 0.0;

        // Pre-calculate constants outside loop
        double dz = goalZMeters - cfg.shooterZMeters;
        double g_d2 = cfg.g * d * d;
        
        // Check speed limits once
        Double minSpeed = cfg.minSpeedMps;
        Double maxSpeed = cfg.maxSpeedMps;

        for (int pass = 0; pass < (useBand ? 2 : 1); pass++) {
            boolean restrictToBand = useBand && pass == 0;

            double bestAngle = Double.NaN;
            double bestV = Double.POSITIVE_INFINITY;
            double bestErr = Double.POSITIVE_INFINITY;

            for (double angle = cfg.minAngleDeg; angle <= cfg.maxAngleDeg + 1e-12; angle += cfg.angleStepDeg) {
                // Inline requiredSpeedForAngle for performance
                double aRad = Math.toRadians(angle);
                double cosA = Math.cos(aRad);
                double tanA = Math.tan(aRad);

                double denom = 2.0 * cosA * cosA * (d * tanA - dz);
                if (denom <= 0.0) continue;

                double v2 = g_d2 / denom;
                if (!(v2 > 0.0) || !Double.isFinite(v2)) continue;
                
                double v = Math.sqrt(v2);

                // Speed limit checks
                if (minSpeed != null && v < minSpeed) continue;
                if (maxSpeed != null && v > maxSpeed) continue;

                double err = Math.abs(v - vRefMps);
                if (restrictToBand && err > band) continue;

                // Update best solution
                if (err < bestErr - 1e-9 || (Math.abs(err - bestErr) <= 1e-9 && v < bestV)) {
                    bestErr = err;
                    bestV = v;
                    bestAngle = angle;
                }
            }

            if (Double.isFinite(bestV)) {
                double aRad = Math.toRadians(bestAngle);
                double vx = bestV * Math.cos(aRad);
                if (vx <= 1e-9) return Solution.invalid("Numerical issue: horizontal velocity ~0.");
                double t = d / vx;

                double wheelRpm = exitSpeedToWheelRpm(bestV, cfg);
                double motorRpm = wheelRpmToMotorRpm(wheelRpm, cfg);

                return Solution.ok(bestAngle, bestV, wheelRpm, motorRpm, t);
            }
        }

        return Solution.invalid("No reachable solution within angle/speed limits.");
    }

    /**
     * Solves ballistic trajectory preferring a constant motor RPM.
     * This is a convenience overload that converts RPM to m/s internally.
     * 
     * @param xMeters Horizontal X distance to target (meters)
     * @param yMeters Horizontal Y distance to target (meters)
     * @param goalZMeters Target height (meters, absolute)
     * @param preferredMotorRpm Desired motor RPM for consistent shots
     * @param cfg Configuration with robot physical constants
     * @return Solution with angle, speeds, and RPMs
     */
    public static Solution solvePreferConstantRpm(
            double xMeters,
            double yMeters,
            double goalZMeters,
            double preferredMotorRpm,
            Config cfg
    ) {
        // Convert preferred motor RPM to exit speed (m/s)
        double vRefMps = motorRpmToExitSpeed(preferredMotorRpm, cfg);
        
        if (!Double.isFinite(vRefMps) || vRefMps <= 0.0) {
            return Solution.invalid("Invalid motor RPM or config parameters.");
        }
        
        // Call the existing method with converted speed
        return solvePreferConstantSpeed(xMeters, yMeters, goalZMeters, vRefMps, cfg);
    }

    // Example usage
    public static void main(String[] args) {
        Config cfg = new Config();
        cfg.shooterZMeters = 0.135;
        cfg.flywheelDiameterMeters = 0.1016; // 4"
        cfg.exitVelocityFactor = 0.85;       // tune this
        cfg.gearRatioMotorToWheel = 2.0;     // example: 2:1 reduction

        double goalZ = 1.8288;
        double vRef = 9.1;

        Solution s = solvePreferConstantSpeed(6.5, 0.0, goalZ, vRef, cfg);
        System.out.println(s);
    }
}