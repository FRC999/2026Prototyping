package frc.robot.lib;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PathPlannerConstants;

public class TrajectoryHelper {

    public static Pose2d flipQuestPoseRed(Pose2d pose) {
        return (PathPlannerConstants.shouldFlipTrajectoryOnRed) ? 
            FlippingUtil.flipFieldPose(pose) :
            pose;
    }
}
