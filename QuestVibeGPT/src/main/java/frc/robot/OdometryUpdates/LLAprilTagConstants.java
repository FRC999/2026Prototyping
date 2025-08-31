package frc.robot.OdometryUpdates;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class LLAprilTagConstants {
    public static final class LLVisionConstants {

    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeAndyMark; // Field Layout - changes year-to-year

		public static enum LLCamera {

			LLLEFT(
				"limelight-fl"
			),

			LLRIGHT(
				"limelight-fr"
			)
			// ,
			// LLBACK(
			// 	"limelight-back"
			// )
			;
			private String cameraname;
			LLCamera(String cn) {
				this.cameraname = cn;
			}
			public String getCameraName() {
				return cameraname;
			}
		}

        public static final double kMaxSingleTagAmbiguity = 0.20; // Maximum ambiguity when seeing a single april tag
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
