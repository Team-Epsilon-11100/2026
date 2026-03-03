// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem responsible for:
 * 1. Capturing AprilTag detections from cameras
 * 2. Estimating robot pose from vision observations
 * 3. Providing pose corrections to the drivetrain odometry
 * 4. Storing the latest vision-estimated pose for subsystem access
 */
public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    
    // Latest vision-estimated robot pose - accessible to all subsystems
    private static Pose2d latestVisionPose = null;
    private static double latestVisionTimestamp = 0.0;
    
    // Static reference to the Vision subsystem instance for accessing camera inputs
    private static Vision instance = null;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Set singleton instance
        instance = this;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the latest vision-estimated robot pose.
     * This is a static method so any subsystem can access it without a Vision reference.
     * 
     * @return The latest vision-estimated robot pose, or null if no valid vision data is available
     */
    public static Pose2d getLatestVisionPose() {
        return latestVisionPose;
    }

    /**
     * Returns the timestamp of the latest vision pose estimate.
     * 
     * @return Timestamp in seconds, or 0.0 if no valid vision data is available
     */
    public static double getLatestVisionTimestamp() {
        return latestVisionTimestamp;
    }

    /**
     * Returns whether vision has a valid pose estimate.
     * 
     * @return True if vision has captured at least one valid pose
     */
    public static boolean hasValidPose() {
        return latestVisionPose != null;
    }

    /**
     * Gets the pose of the closest currently-visible AprilTag.
     * Uses drivetrain odometry to calculate distances to all visible tags.
     * 
     * @param robotPose Current robot pose from drivetrain odometry
     * @return Pose3d of the closest visible AprilTag, or null if no tags visible
     */
    public static Pose3d getClosestVisibleTag(Pose2d robotPose) {
        // Need valid instance to access camera inputs
        if (instance == null || robotPose == null) {
            return null;
        }

        Pose3d closestTag = null;
        double closestDistance = Double.MAX_VALUE;

        // Check all cameras for visible tags
        for (int i = 0; i < instance.inputs.length; i++) {
            if (!instance.inputs[i].connected) {
                continue;
            }

            // Check all visible tag IDs from this camera
            for (int tagId : instance.inputs[i].tagIds) {
                // Get tag pose from field layout
                var tagPoseOpt = constVision.aprilTagLayout.getTagPose(tagId);
                if (tagPoseOpt.isEmpty()) {
                    continue;  // Invalid tag ID
                }

                Pose3d tagPose = tagPoseOpt.get();
                
                // Calculate distance from robot to tag (2D floor distance)
                double distance = Math.hypot(
                    tagPose.getX() - robotPose.getX(),
                    tagPose.getY() - robotPose.getY()
                );

                // Update closest tag
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestTag = tagPose;
                }
            }
        }

        return closestTag;
    }

    /**
     * Gets the pose of the closest currently-visible AprilTag.
     * Uses the latest vision pose to calculate distances to all visible tags.
     * DEPRECATED: Use getClosestVisibleTag(Pose2d robotPose) instead for more reliable operation.
     * 
     * @return Pose3d of the closest visible AprilTag, or null if no tags visible or no robot pose
     */
    @Deprecated
    public static Pose3d getClosestVisibleTag() {
        // Fallback to vision pose if available
        if (latestVisionPose == null) {
            return null;
        }
        return getClosestVisibleTag(latestVisionPose);
    }

    /**
     * Main periodic method:
     * - Reads camera observations from all cameras
     * - Filters invalid detections
     * - Estimates robot pose from AprilTag observations
     * - Sends pose corrections to drivetrain odometry (automatically merged by WPILib)
     * - Stores latest pose for subsystem access
     * 
     * Data Merging Strategy:
     * Each camera independently sends pose measurements to the drivetrain's pose estimator.
     * WPILib's SwerveDrivePoseEstimator automatically fuses multiple vision measurements
     * with wheel odometry using a Kalman filter. Each measurement is weighted by its
     * standard deviation - more confident measurements (lower stddev) have more influence.
     * 
     * Multi-camera benefits:
     * - Redundancy: If one camera loses sight of tags, others may still see them
     * - Coverage: Multiple viewing angles provide better field coverage
     * - Accuracy: Independent measurements from different angles improve overall accuracy
     */
    @Override
    public void periodic() {
        // Update all camera inputs
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Track all accepted poses for logging
        List<Pose3d> allAcceptedPoses = new LinkedList<>();
        
        // Log camera connection status and tag counts for debugging
        int totalVisibleTags = 0;
        for (int i = 0; i < inputs.length; i++) {
            if (inputs[i].connected) {
                totalVisibleTags += inputs[i].tagIds.length;
            }
        }
        SmartDashboard.putNumber("Vision/TotalVisibleTags", totalVisibleTags);
        SmartDashboard.putNumber("Vision/ConnectedCameras", 
            (int) java.util.Arrays.stream(inputs).filter(input -> input.connected).count());

        // Process each camera
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Skip if camera disconnected
            if (!inputs[cameraIndex].connected) {
                continue;
            }

            // Track accepted poses for this camera
            List<Pose3d> cameraAcceptedPoses = new LinkedList<>();

            // Process each pose observation from this camera
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Null check
                if (observation == null || observation.pose() == null) {
                    continue;
                }

                // ===== FILTERING: Reject invalid observations =====
                boolean rejectPose = 
                    observation.tagCount() == 0 ||  // Must have at least one tag
                    (observation.tagCount() == 1 && observation.ambiguity() > constVision.maxAmbiguity) || // Single tag with high ambiguity
                    Math.abs(observation.pose().getZ()) > constVision.maxZError || // Unrealistic Z height
                    observation.pose().getX() < 0.0 ||  // Outside field boundaries
                    observation.pose().getX() > constVision.aprilTagLayout.getFieldLength() ||
                    observation.pose().getY() < 0.0 ||
                    observation.pose().getY() > constVision.aprilTagLayout.getFieldWidth();

                // Debug: Log why poses are rejected
                if (rejectPose) {
                    if (observation.tagCount() == 0) {
                        SmartDashboard.putString("Vision/Camera" + cameraIndex + "/Reject", "No tags");
                    } else if (observation.tagCount() == 1 && observation.ambiguity() > constVision.maxAmbiguity) {
                        SmartDashboard.putString("Vision/Camera" + cameraIndex + "/Reject", 
                            "Ambiguity: " + String.format("%.2f", observation.ambiguity()));
                    } else if (Math.abs(observation.pose().getZ()) > constVision.maxZError) {
                        SmartDashboard.putString("Vision/Camera" + cameraIndex + "/Reject", 
                            "Z error: " + String.format("%.2fm", observation.pose().getZ()));
                    } else {
                        SmartDashboard.putString("Vision/Camera" + cameraIndex + "/Reject", "Out of bounds");
                    }
                    continue;  // Skip rejected observations
                } else {
                    SmartDashboard.putString("Vision/Camera" + cameraIndex + "/Reject", "Accepted");
                }

                // ===== POSE ESTIMATION: Convert 3D pose to 2D floor pose =====
                // Extract only yaw rotation, ignore roll and pitch
                var originalPose = observation.pose();
                double yawRadians = originalPose.getRotation().getZ();

                var floorPose2d = new Pose2d(
                    originalPose.getX(),
                    originalPose.getY(),
                    new Rotation2d(yawRadians)  // Only yaw, no roll/pitch
                );

                // Convert back to 3D for logging (Z=0, no roll/pitch)
                var floorPose3d = new Pose3d(
                    floorPose2d.getX(),
                    floorPose2d.getY(),
                    0.0,  // On floor
                    new Rotation3d(0, 0, yawRadians)  // Only yaw
                );

                // Add to accepted poses
                cameraAcceptedPoses.add(floorPose3d);

                // ===== UPDATE STORED POSE: Track latest vision estimate =====
                if (observation.timestamp() > latestVisionTimestamp) {
                    latestVisionPose = floorPose2d;
                    latestVisionTimestamp = observation.timestamp();
                }

                // ===== CALCULATE STANDARD DEVIATIONS: Determine confidence =====
                double avgDistance = Math.max(0.1, observation.averageTagDistance());
                int tagCount = Math.max(1, observation.tagCount());

                double stdDevFactor = Math.pow(avgDistance, 2.0) / tagCount;
                double linearStdDev = constVision.linearStdDevBaseline * stdDevFactor;
                double angularStdDev = constVision.angularStdDevBaseline * stdDevFactor;

                // Apply MegaTag 2 multipliers if applicable
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= constVision.linearStdDevMegatag2Factor;
                    angularStdDev *= constVision.angularStdDevMegatag2Factor;
                }

                // Apply camera-specific multipliers
                if (cameraIndex < constVision.cameraStdDevFactors.length) {
                    linearStdDev *= constVision.cameraStdDevFactors[cameraIndex];
                    angularStdDev *= constVision.cameraStdDevFactors[cameraIndex];
                }

                // ===== SEND TO ODOMETRY: Provide pose correction =====
                consumer.accept(
                    floorPose2d,
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log accepted poses for this camera
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/AcceptedPoses",
                cameraAcceptedPoses.toArray(new Pose3d[0])
            );

            allAcceptedPoses.addAll(cameraAcceptedPoses);
        }

        // ===== LOGGING: Summary data =====
        Logger.recordOutput("Vision/AllAcceptedPoses", allAcceptedPoses.toArray(new Pose3d[0]));
        
        if (latestVisionPose != null) {
            Logger.recordOutput("Vision/LatestPose", new Pose3d(
                latestVisionPose.getX(),
                latestVisionPose.getY(),
                0.0,
                new Rotation3d(0, 0, latestVisionPose.getRotation().getRadians())
            ));
            Logger.recordOutput("Vision/LatestPoseTimestamp", latestVisionTimestamp);
        }
    }

    /**
     * Functional interface for sending vision measurements to drivetrain odometry.
     * The drivetrain should implement this to receive pose corrections.
     */
    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }
}
