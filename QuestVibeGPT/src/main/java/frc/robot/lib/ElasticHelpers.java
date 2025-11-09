package frc.robot.lib;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

public class ElasticHelpers {
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

}