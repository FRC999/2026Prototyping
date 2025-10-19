package frc.robot.OdometryUpdates;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestNavConstants {
    public static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.23, -0.26, Rotation2d.fromDegrees(-41)); //Original was -0.32, -0.29
    // Transform2d ROBOT_TO_QUEST = new Transform2d(0.4, 0.3, Rotation2d.k180deg); 2024 Constant
    public static final Pose2d characterizationQuestPose = new Pose2d();
    public static final Pose2d nullPose = new Pose2d(-1000, -1000, Rotation2d.kZero);
    public static final Pose2d robotZeroPose = new Pose2d(0, 0, Rotation2d.kZero);

    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill( 
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );


}
