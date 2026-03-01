package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.constHood;

public class BallisticSolver {

    public static class Config {
        public double shooterZMeters = 0.135;
        public double g = 9.806;

        public double minAngleDeg = constHood.minHoodAngleDegrees; // example: 20.0
        public double maxAngleDeg = constHood.maxHoodAngleDegrees; // example: 56.0
        public double angleStepDeg = 0.05;

        public Double minSpeedMps = null;
        public Double maxSpeedMps = null;

        public Double preferSpeedDeltaMps = null; // e.g. 0.25

        // --- Flywheel / drivetrain conversion ---
        public double flywheelDiameterMeters = Units.inchesToMeters(4); // 4 inches = 0.1016 m
        /**
         * k = (ball exit speed) / (wheel surface speed)
         * so wheelSurfaceSpeed = exitSpeed / k
         * Start with ~0.85 and tune from real shots.
         */
        public double exitVelocityFactor = 0.85;

        /**
         * gearRatioMotorToWheel = motorRPM / wheelRPM
         * Example: 2:1 reduction (motor spins 2x wheel) => 2.0
         * Example: 1:2 overdrive (motor spins half of wheel) => 0.5
         */
        public double gearRatioMotorToWheel = 1.0;
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

        for (int pass = 0; pass < (useBand ? 2 : 1); pass++) {
            boolean restrictToBand = useBand && pass == 0;

            double bestAngle = Double.NaN;
            double bestV = Double.POSITIVE_INFINITY;
            double bestErr = Double.POSITIVE_INFINITY;

            for (double angle = cfg.minAngleDeg; angle <= cfg.maxAngleDeg + 1e-12; angle += cfg.angleStepDeg) {
                double v = requiredSpeedForAngle(d, goalZMeters, angle, cfg);
                if (!Double.isFinite(v)) continue;

                if (cfg.minSpeedMps != null && v < cfg.minSpeedMps) continue;
                if (cfg.maxSpeedMps != null && v > cfg.maxSpeedMps) continue;

                double err = Math.abs(v - vRefMps);
                if (restrictToBand && err > band) continue;

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