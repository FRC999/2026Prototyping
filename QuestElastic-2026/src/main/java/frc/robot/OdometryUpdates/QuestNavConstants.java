package frc.robot.OdometryUpdates;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestNavConstants {
    //public static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.23, -0.26, Rotation2d.fromDegrees(-41)); //Original was -0.32, -0.29
    public static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.26, -0.24, Rotation2d.fromDegrees(-53.45)); //2024 Constant
    public static final Transform3d ROBOT_TO_QUEST_3D = new Transform3d(0.26, -0.24, 0.0, new Rotation3d(Rotation2d.fromDegrees(-53.45))); //2024 Constant
    public static final Pose2d characterizationQuestPose = new Pose2d();
    public static final Pose3d characterizationQuestPose3d = new Pose3d(characterizationQuestPose);
    public static final Pose2d nullPose = new Pose2d(-1000, -1000, Rotation2d.kZero);
    public static final Pose3d nullPose3d = new Pose3d(-1000, -1000, -1000, Rotation3d.kZero);
    public static final Pose2d robotZeroPose = new Pose2d(0, 0, Rotation2d.kZero);
    public static final Pose3d robotZeroPose3d = new Pose3d(0, 0, 0, Rotation3d.kZero);

    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill( 
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    public static final double llWait = 1000.0;
    public static Pose2d startingPositionNoLL = new Pose2d(10.331, 6.31, new Rotation2d(0));

}
