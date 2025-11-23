package frc.robot.lib;

import java.util.ArrayList;

import javax.xml.crypto.dsig.spec.ExcC14NParameterSpec;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PathPlannerConstants;

public class TrajectoryHelper {

    private static final double FIELD_LENGTH_BLUE = 8.2296; // 27 feet

    private ArrayList<PathPlannerPath> blueTrajectories = new ArrayList<PathPlannerPath>();
    private ArrayList<PathPlannerPath> redTrajectories = new ArrayList<PathPlannerPath>();

    public TrajectoryHelper() {
        try {
            blueTrajectories.add(PathPlannerPath.fromPathFile("ElasticTestPath"));
            blueTrajectories.add(PathPlannerPath.fromPathFile("ThreeMeterForward"));
            blueTrajectories.add(PathPlannerPath.fromPathFile("OneMeterForwardTurn"));

            redTrajectories.add(PathPlannerPath.fromPathFile("RedElasticTestPath"));
            redTrajectories.add(PathPlannerPath.fromPathFile("RedThreeMeterForward"));
            redTrajectories.add(PathPlannerPath.fromPathFile("RedOneMeterForwardTurn"));
        } catch (Exception e) {
            
        }
    }

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

    public static PathPlannerPath selectAutoBasedOnATPose(Pose2d robotPose2d) {
        boolean isRobotBlue = true;
        if (robotPose2d.getX() < FIELD_LENGTH_BLUE / 2) {
            isRobotBlue = true;
        } else {
            isRobotBlue = false;
        }

        if(isRobotBlue) {
            return nearestTrajectoryBlue();
        } else {
            return nearestTrajectoryRed();
        }
    }

    private static PathPlannerPath nearestTrajectoryBlue() {
        return null;
    }

    private static PathPlannerPath nearestTrajectoryRed() {
        return null;
    }
}
