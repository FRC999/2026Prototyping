package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotContainer;

public class ElasticHelpers {

    private static final Field2d field = new Field2d();
    private static Pose2d currentPose = new Pose2d();
    private static Trajectory currentTrajectory = new Trajectory();

    public static String questStatesColors(String state) {
        switch (state) {
            case "INITIALIZE":
                return "#4E5FFF";
            case "SEEKING_TAGS_Q":
                return "#CF4EFF";
            case "SEEKING_TAGS_NO_Q":
                return "#E22222";
            case "CALIBRATED_Q":
                return "#43D567";
            case "CALIBRATED_NO_Q":
                return "#EDFF4E";
            default:
                return "#000000";
        }
    }

    public static String LLAnyVisibleColors(boolean visible) {
        if (visible) {
            return "#43D567";
        }
        return "#E22222";
    }

    public static String LLBestAmbiguityColors(double ambiguity, double maxAmbiguity) {
        if (ambiguity <= maxAmbiguity) {
            return "#43D567";
        }
        return "#EDFF4E";
    }

    public static String getAllianceSide() {
        return DriverStation.getAlliance().map(alliance -> {
            switch (alliance) {
                case Red:
                    return "#FF0000";
                case Blue:
                    return "#0000FF";
                default:
                    return "Invalid";
            }
        }).orElse("Invalid");
    }

    public static String getAutoSelectedColor() {
        try {  
            String autoSelected = RobotContainer.autoChooser.getSelected().toString();
            // System.out.println("Auto Selected: " + autoSelected);
            if (autoSelected.contains("RED")) {
                return "#FF0000";
            } else if (autoSelected.contains("BLUE")) {
                return "#0000FF";
            } else {
                return "#00FF00"; // Green for other selections
            }
        } catch (Exception e) {
            return "Uh Oh oops: " + e;
        }
        
    }

    public static boolean shouldEndGame() { 
        double matchTime = DriverStation.getMatchTime();
        return matchTime <= 30.0;
    }

    public static String shouldEndGameColor() {
        if (shouldEndGame()) {
            return "#FF00d0"; // Red
        } else {
            return "#00FF00"; // Green
        }
    }

    // Returns the global Field2d instance 
    public static Field2d getField() {
        return field;
    }

    // Updates the robot pose displayed on the field 
    public static void updateRobotPose(Pose2d pose) {
        currentPose = pose;
        field.setRobotPose(pose);
    }

    // Returns the most recently stored robot pose 
    public static Pose2d getCurrentPose() {
        return currentPose;
    }

    // Displays a trajectory on the field 
    public static void setTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        field.getObject("Trajectory").setTrajectory(trajectory);
    }

    // Clears any displayed trajectory 
    public static void clearTrajectory() {
        field.getObject("Trajectory").setPoses();
    }

}