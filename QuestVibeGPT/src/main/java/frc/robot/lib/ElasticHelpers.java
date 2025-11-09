package frc.robot.lib;

public class ElasticHelpers {
    public static String questStatesColors(String state){
        switch (state){
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

    public static String LLAnyVisibleColors(boolean visible){
        if (visible){
            return "#43D567";
        }
        return "#E22222";
    }
}