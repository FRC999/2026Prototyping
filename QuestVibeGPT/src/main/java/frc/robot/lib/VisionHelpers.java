package frc.robot.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Optional;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.OdometryUpdates.LLAprilTagConstants.VisionHelperConstants;
import frc.robot.OdometryUpdates.LLAprilTagConstants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.LimelightHelpers.PoseEstimate;
import frc.robot.OdometryUpdates.LLAprilTagConstants;
import frc.robot.OdometryUpdates.LLAprilTagSubsystem;
import edu.wpi.first.math.geometry.Pose3d;


/** Add your docs here. */
public class VisionHelpers {

    public static List<Pose2d> apriltagPoses = new ArrayList<>();

    public static double getDistanceBetweenCenterCameraCoral(Double alpha, Double beta, double h, double d) {
        if(!alpha.isNaN() && beta.isNaN()){
            return calculateDistanceUsingAlpha(alpha, h, d);
        } else if (alpha.isNaN() && !beta.isNaN()){
            return calculateDistanceUsingAlpha(beta, h, d);
        } else if(!alpha.isNaN() && !beta.isNaN()){
            if(Math.abs(alpha)>beta){
                return calculateDistanceUsingBeta(beta, h, d);
            } else {
                return calculateDistanceUsingAlpha(alpha, h, d);
            }
        }
        return 0;
    }

    public static double calculateDistanceUsingAlpha(Double alpha, double h, double d){
        double x = (h-Math.tan(90 - Math.abs(alpha))*d/2.0)/Math.tan(90 - Math.abs(alpha));
        return x;
    }

    public static double calculateDistanceUsingBeta(Double beta, double h, double d){
        double x = d/2.0 - h*Math.tan(Math.abs(beta));
        return x;
    }

    public static double calculateTheta (double d, Double alpha, Double beta, double h ) {
        double c = (Math.sin(90-beta) * d)/Math.sin(Math.abs(alpha) * Math.abs(beta));
        double x = (Math.sin(90 - Math.abs(alpha)) * c);
        double l = (x / Math.tan(90 - Math.abs(alpha)));
        double y = (-(l * x)/ h) + ((d * x) / (2 * h)); 
        double z = 1 - (x / h);
        double totalLength = y/z;
        return Math.atan(x / totalLength); 
    }

    
        
    /**
     * Get the pose of a specific AprilTag by its ID.
     *
     * @param tagId The ID of the AprilTag.
     * @return The pose of the AprilTag as a Pose3d, or null if the tag is not found.
     */

    

    public static Pose3d getTagPose(int tagId) {
        
        if (LLAprilTagSubsystem.fieldLayout != null) {
            Optional<Pose3d> tagPose = LLAprilTagSubsystem.fieldLayout.getTagPose(tagId);
            if (tagPose.isPresent()) {
                return tagPose.get();
            } else {
                DriverStation.reportWarning("AprilTag " + tagId + " not found in the field layout.", false);
            }
        }
        return null;
    }

    /**
     * Coral Station sides are LOWER and HIGHER. That means for BLUE the LOWER one is on the RIGHT and HIGHER is on the LEFT
     * For RED it's opposite - LOWER on the LEFT and HIGHER on the RIGHT from driver point of view
     * 
     * Reef sides are:
     * 1 - facing the driver on the corresponding side (tags 18 for BLUE and 7 for RED)
     * 2 - RED - COUNTERCLOCKWISE from it from driver point of view (tags 8 for RED)
     * same for sides 3,4,5,6
     * BLUE - CLOCKWISE from it from driver point of view (tags 19 for BLUE)
     */
    public static void createHashMapOfTags() {
        RobotPoseConstants.visionRobotPoses.put("TagRedCoralLOW", getTagPose(1).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedCoralHIGH", getTagPose(2).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef1", getTagPose(7).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef2", getTagPose(8).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef3", getTagPose(9).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef4", getTagPose(10).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef5", getTagPose(11).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedReef6", getTagPose(6).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedBarge",  getTagPose(5).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedProcessor",  getTagPose(3).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluCoralLOW", getTagPose(12).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluCoralHIGH", getTagPose(13).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluReef1", getTagPose(18).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluReef2", getTagPose(19).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluReef3", getTagPose(20).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluReef4", getTagPose(21).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluReef5", getTagPose(22).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluReef6", getTagPose(17).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluBarge",  getTagPose(14).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluProcessor",  getTagPose(16).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagBluBargeByProcessor",  getTagPose(15).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("TagRedBargeByProcessor",  getTagPose(4).toPose2d());
    
        // These poses are already transformed with 180 rotation to robot orientation
        for (int i=6;i<=11;i++){
            RobotPoseConstants.reefTagPoses.put(getTagPose(i).toPose2d().plus(new Transform2d(0, 0, Rotation2d.k180deg)),i);
            RobotPoseConstants.redReefTagPoses.put(getTagPose(i).toPose2d().plus(new Transform2d(0, 0, Rotation2d.k180deg)),i);
        }
        for (int i=17;i<=22;i++){
            RobotPoseConstants.reefTagPoses.put(getTagPose(i).toPose2d().plus(new Transform2d(0, 0, Rotation2d.k180deg)),i);
            RobotPoseConstants.blueReefTagPoses.put(getTagPose(i).toPose2d().plus(new Transform2d(0, 0, Rotation2d.k180deg)),i);
        }
    
        apriltagPoses.addAll(RobotPoseConstants.reefTagPoses.keySet());

        // alex test

        // for (int i=0;i<apriltagPoses.size();i++) {
        //     System.out.println("I:"+i+" P "+apriltagPoses.get(i));
        // }
    }

    public static Pose2d getClosestReefTagToRobot(Pose2d robotPose) {
        return robotPose.nearest(VisionHelpers.apriltagPoses);
    }

    public static void mapTagIDToTagKey() {
        RobotPoseConstants.tagNumberToKey.put(1,"TagRedCoralLOW");
        RobotPoseConstants.tagNumberToKey.put(2,"TagRedCoralHIGH");
        RobotPoseConstants.tagNumberToKey.put(7,"TagRedReef1");
        RobotPoseConstants.tagNumberToKey.put(8,"TagRedReef2");
        RobotPoseConstants.tagNumberToKey.put(9,"TagRedReef3");
        RobotPoseConstants.tagNumberToKey.put(10,"TagRedReef4");
        RobotPoseConstants.tagNumberToKey.put(11,"TagRedReef5");
        RobotPoseConstants.tagNumberToKey.put(6, "TagRedReef6");
        RobotPoseConstants.tagNumberToKey.put(5, "TagRedBarge");
        RobotPoseConstants.tagNumberToKey.put(3, "TagRedProcessor");
        RobotPoseConstants.tagNumberToKey.put(12, "TagBluCoralLOW");
        RobotPoseConstants.tagNumberToKey.put(13, "TagBluCoralHIGH");
        RobotPoseConstants.tagNumberToKey.put(18, "TagBluReef1");
        RobotPoseConstants.tagNumberToKey.put(19, "TagBluReef2");
        RobotPoseConstants.tagNumberToKey.put(20, "TagBluReef3");
        RobotPoseConstants.tagNumberToKey.put(21, "TagBluReef4");
        RobotPoseConstants.tagNumberToKey.put(22, "TagBluReef5");
        RobotPoseConstants.tagNumberToKey.put(17, "TagBluReef6");
        RobotPoseConstants.tagNumberToKey.put(14, "TagBluBarge");
        RobotPoseConstants.tagNumberToKey.put(16, "TagBluProcessor");
        RobotPoseConstants.tagNumberToKey.put(15, "TagBluBargeByProcessor");
        RobotPoseConstants.tagNumberToKey.put(4, "TagRedBargeByProcessor");
    }

    public static String getLeftReefName(int id) {
        String tagName = "Robot" + RobotPoseConstants.tagNumberToKey.get(id).substring(3) + "Left";
        return tagName;
    }

    public static String getAlgaeReefName(int id) {
        String tagName = "Robot" + RobotPoseConstants.tagNumberToKey.get(id).substring(3) + "Algae";
        return tagName;
    }

    public static String getRightReefName(int id) {
        String tagName = "Robot" + RobotPoseConstants.tagNumberToKey.get(id).substring(3) + "Right";
        return tagName;
    }



    /**
     * Move the pose in a pose-centric (relative) way by X and Y without changing rotation
     * (e.g. if pose points LEFT (Rotation 90 degrees), the move of 0,1 moves the Y of the pose by +1)
     * The Rotation of the pose will not change
     * @param pose
     * @return
     */
    public static Pose2d movePoseXY(Pose2d pose, double x, double y) {
        return pose.transformBy(new Transform2d(x,y,Rotation2d.kZero));
    }

    // Create positions for robot placement near reef pipes
    // posenames similar to "RobotBluReef1Left"
    // adds to RobotPoseConstants.visionRobotPoses
    public static void addRobotPosesForCoralPlacement() {


        List<String> keys = new ArrayList<>();
        for(String k : RobotPoseConstants.visionRobotPoses.keySet()) {
            keys.add(k);
        }
        for (String key : keys) { // Fill robot poses
            if (key.contains("Reef") && key.contains("Tag")) {
                RobotPoseConstants.visionRobotPoses.put( // e.g. RobotBluReef1Left
                    "Robot" + key.substring(3,6) + "Reef" + 
                        key.substring(10,11) + "Left", // Reef Side //RedReef1Left",
                    movePoseXY( //e.g. RobotRedReef1Left
                        RobotPoseConstants.visionRobotPoses.get(key)
                            .plus(new Transform2d(0, 0, Rotation2d.k180deg)) // Tag poses look TOWARDS the bot, so need to reverse them 180 degrees for the bot direction placement
                        //, -(VisionHelperConstants.bumperWidth + (SwerveChassis.WHEEL_BASE / 2.0)
                        , (-0.42
                        ) // Coordinates of the center of the bot, so need to move them back half-length of the bot
                        , VisionHelperConstants.distanceBetweenReefPoles / 2.0 // Move bot to the left
                    )
                );
                RobotPoseConstants.visionRobotPoses.put( // e.g. RobotBluReef1Left
                    "Robot" + key.substring(3,6) + "Reef" + 
                        key.substring(10,11) + "Right", // Reef Side //RedReef1Left",
                    movePoseXY( //e.g. RobotRedReef1Left
                        RobotPoseConstants.visionRobotPoses.get(key)
                            .plus(new Transform2d(0, 0, Rotation2d.k180deg)) // Tag poses look TOWARDS the bot, so need to reverse them 180 degrees for the bot direction placement
                        //, -(VisionHelperConstants.bumperWidth + (SwerveChassis.WHEEL_BASE / 2.0)
                        , (-0.42
                        ) // Coordinates of the center of the bot, so need to move them back half-length of the bot
                        , -VisionHelperConstants.distanceBetweenReefPoles / 2.0 // Move bot to the left
                    )
                );

            } else if ( (key.contains("Coral") || key.contains("Processor"))
                && key.contains("Tag")) {
                    RobotPoseConstants.visionRobotPoses.put( // e.g. RobotBluReef1Left
                        "Robot" + key.substring(3),
                    movePoseXY( //e.g. RobotRedReef1Left
                        RobotPoseConstants.visionRobotPoses.get(key)
                            .plus(new Transform2d(0, 0, Rotation2d.k180deg)) // Tag poses look TOWARDS the bot, so need to reverse them 180 degrees for the bot direction placement
                        //, -(VisionHelperConstants.bumperWidth + (SwerveChassis.WHEEL_BASE / 2.0)
                        ,-(0.47
                        ) // Coordinates of the center of the bot, so need to move them back half-length of the bot
                        , 0
                    )
                );
            }

        }
        
        // Special poses we determined manually
        RobotPoseConstants.visionRobotPoses.put("RobotBlueStationUp", new Pose2d(1.458, 7.255, Rotation2d.fromDegrees(-55.0)));
        RobotPoseConstants.visionRobotPoses.put("RobotBlueStationDown", new Pose2d(0.638, 0.630, Rotation2d.fromDegrees(-55.0)));
        RobotPoseConstants.visionRobotPoses.put("RobotRedStationDown", new Pose2d(16.690, 0.630, Rotation2d.fromDegrees(126.0))); //TODO: needs to be changed
        RobotPoseConstants.visionRobotPoses.put("RobotRedStationUp", new Pose2d(16.742, 7.307, Rotation2d.fromDegrees(126.0))); //TODO: needs to be changed

        // alex test
        // List<String> keys2 = new ArrayList<>();
        // for (String k : RobotPoseConstants.visionRobotPoses.keySet()) {
        //   keys2.add(k);
        // }
        // System.out.println("===---");
        //  for (String key : keys2) {
        //    System.out.println(key + " : " + RobotPoseConstants.visionRobotPoses.get(key)
        //    );
        //  }
        
         //System.out.println(RobotPoseConstants.visionRobotPoses);
    }

    public static void addRobotPosesForAlgaePlacement() {
        List<String> keys = new ArrayList<>();
        for(String k : RobotPoseConstants.visionRobotPoses.keySet()) {
            keys.add(k);
        }
        for (String key : keys) {
        if (key.contains("Reef") && key.contains("Tag")) {
            RobotPoseConstants.visionRobotPoses.put( // e.g. RobotBluReef1Algae
                "Robot" + key.substring(3,6) + "Reef" + 
                    key.substring(10,11) + "Algae", // Reef Side //RedReef1Algae",
                movePoseXY( //e.g. RobotRedReef1Left
                    RobotPoseConstants.visionRobotPoses.get(key)
                        .plus(new Transform2d(0, 0, Rotation2d.k180deg)) // Tag poses look TOWARDS the bot, so need to reverse them 180 degrees for the bot direction placement
                    //, -(VisionHelperConstants.bumperWidth + (SwerveChassis.WHEEL_BASE / 2.0)
                    , (-0.42
                    ) // Coordinates of the center of the bot, so need to move them back half-length of the bot
                    , 0 // Move bot to the left
                )
            );

        } 
        }
    }

    /**
     * Check if the AprilTag field layout was loaded successfully.
     *
     * @return True if the field layout is valid, false otherwise.
     */
    public static boolean isFieldLayoutValid() {
        return LLAprilTagSubsystem.fieldLayout != null;
    }

    public static Pose2d getKeyByValue(HashMap<Pose2d, Integer> map, int value) {
        for (Map.Entry<Pose2d,Integer> entry : map.entrySet()) {
            if ( entry.getValue() == value) {
                return entry.getKey();
            }
        }
        return null;
    }

    /**
     * Check if a robot is on the left or right of the apriltag plane as determined by the apriltag vector (should point from front to back of the tag)
     * @param tagPose
     * @param robotPose
     * @param tolerance
     * @return
     */
    public static int amIOntheLeftOrRightOfThePose(Pose2d tagPose, Pose2d robotPose,double tolerance) {
        Pose2d rotatedRobotPose = robotPose.rotateAround(tagPose.getTranslation(), tagPose.getRotation().unaryMinus());
    
        if ( Math.abs(tagPose.getY()-rotatedRobotPose.getY()) < tolerance ) {
          return 0; // I am there!
        } else if (tagPose.getY()-rotatedRobotPose.getY() < 0) {
          return 1;
        } else { return -1;}
    }

    public static double distanceFromPoseLineY(Pose2d tagPose, Pose2d robotPose) {
        Pose2d rotatedRobotPose = robotPose.rotateAround(tagPose.getTranslation(), tagPose.getRotation().unaryMinus());
        return tagPose.getY()-rotatedRobotPose.getY();
    }
    public static double distanceFromPoseLineX(Pose2d tagPose, Pose2d robotPose) {
        Pose2d rotatedRobotPose = robotPose.rotateAround(tagPose.getTranslation(), tagPose.getRotation().unaryMinus());
        return tagPose.getX()-rotatedRobotPose.getX();
    }

    public static void updateLLTelemetry(PoseEstimate pe, String cn) {
        try { // Errors in tag determination can kill robot
          if (DebugTelemetrySubsystems.ll) { // Telemetry for LL AT recognition
            SmartDashboard.putNumber("LL " + cn + " PoseEst TagCount", pe.tagCount);
            SmartDashboard.putNumber("LL " + cn + " PoseEst Ambiguity", pe.rawFiducials[0].ambiguity);
            SmartDashboard.putNumber("LL " + cn + " Tag Number", pe.rawFiducials[0].id);
            SmartDashboard.putString("LL " + cn + " Tag Pose",
                VisionHelpers.getTagPose(pe.rawFiducials[0].id).toString());
          }
        } catch (Exception e) {

        }
    }

    public static void clearLLTelemetry(String cn) {
        if (DebugTelemetrySubsystems.ll) {
            SmartDashboard.putNumber("LL " + cn + " PoseEst TagCount", 0);
            SmartDashboard.putNumber("LL " + cn + " PoseEst Ambiguity", 10);
            SmartDashboard.putNumber("LL " + cn + " Tag Number", 0);
            SmartDashboard.putString("LL " + cn + " Tag Pose", "");
        }
    }


}
