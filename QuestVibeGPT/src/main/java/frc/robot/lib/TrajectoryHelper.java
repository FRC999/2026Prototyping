package frc.robot.lib;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PathPlannerConstants;

public class TrajectoryHelper {

    /**
     * When running a trajectory, PP flips it automatically (unless disabled by a flag)
     * around field center. To recalibrate Quest to the flipped trajectory, this method
     * may be used to flip the initial pose of the trajectory.
     * This is only needed if odometry reset to the starting pose is needed (e.g. if cameras
     * could not calibrate quest at the beginning of the game for some reason, so we
     * have to assume starting pose of the bot)
     * @param pose
     * @return pose flipped around center of the field
     */
    public static Pose2d flipQuestPoseRed(Pose2d pose) {
        return (PathPlannerConstants.shouldFlipTrajectoryOnRed) ? 
            FlippingUtil.flipFieldPose(pose) :
            pose;
    }
}
