package frc.robot.OdometryUpdates;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class LLAprilTagConstants {
    public static final class LLVisionConstants {

    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeAndyMark; // Field Layout - changes year-to-year
	
	/** Limelight IMU mode for MT2: 0=external only, 1=external seeds LL IMU, 2=internal. */
	public static final int LL_IMU_MODE = 1;
		
	public static enum LLCamera {

			LLLEFT(
				"limelight-fl"
			),

			LLRIGHT(
				"limelight-fr"
			),
			LLLEFTUP (
				"limelight-l"
			)
			// ,
			// LLBACK(
			// 	"limelight-back"
			// )
			;
			private String cameraname;
			private boolean prevCleared; // Set to true if nothing was seen last time
			public boolean isPrevCleared() {
				return prevCleared;
			}
			public void setPrevCleared(boolean prevCleared) {
				this.prevCleared = prevCleared;
			}
			LLCamera(String cn) {
				this.cameraname = cn;
				this.prevCleared = false;
			}
			public String getCameraName() {
				return cameraname;
			}
			
		}

        public static final double kMaxQuestCalibrationAmbiguity = 0.30;
        public static final double kMaxSingleTagAmbiguity = 0.20; // Maximum ambiguity when seeing a single AprilTag
        public static final double kMaxCameraToTargetDistance = 3.0; //Maximum distance from camera to AprilTag
	}

    public static final class VisionHelperConstants {
		public static final double distanceBetweenReefPoles = Units.inchesToMeters(12.5); // page 162 https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
		public static final double bumperWidth = Units.inchesToMeters(2.5);
		public static class RobotPoseConstants {
			public static Map<String, Pose2d> visionRobotPoses = new HashMap<String, Pose2d>();
			public static Map<Integer, String> tagNumberToKey = new HashMap<Integer, String>();
			public static Map<Pose2d, Integer> reefTagPoses = new HashMap<Pose2d, Integer>();
			public static Map<Pose2d, Integer> redReefTagPoses = new HashMap<Pose2d, Integer>();
			public static Map<Pose2d, Integer> blueReefTagPoses = new HashMap<Pose2d, Integer>();
		}
	}
}
