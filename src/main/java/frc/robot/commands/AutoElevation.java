package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.BallisticSolver;
import frc.robot.utils.BallisticSolver.Solution;

public class AutoElevation {
   

    double angle;
    double rpm;
    double time;
    double angleRad;
    Hood hood;
    Flywheel flywheel;
    public AutoElevation(Flywheel flywheel, Hood hood) {
        this.hood = hood;
        this.flywheel = flywheel;

        Pose2d latestPose = Vision.getLatestVisionPose();
        Pose3d targetPose = Vision.getClosestVisibleTag();
        Solution s = BallisticSolver.solvePreferConstantSpeed(latestPose.getX() - targetPose.getX(),
                latestPose.getY() - targetPose.getY(), targetPose.getZ(), 9.1, null);

        if (s.valid()) { 
            angle = s.angleDeg();
            rpm = s.motorRpm();
            time = s.timeSec();
            angleRad = s.angleRad();
        } else {
            System.out.println(s.reason());
        }

        hood.setHoodAngle(angle);
        flywheel.setFlywheelRpm(rpm);
    }


}
