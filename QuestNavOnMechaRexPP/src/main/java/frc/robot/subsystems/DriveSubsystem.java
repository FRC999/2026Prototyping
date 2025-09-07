// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveConstants.TunerConstants;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

  Pigeon2 imu;
  private double previousOmegaRotationCommand;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveChassis.MaxSpeed * 0.1).withRotationalDeadband(SwerveChassis.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
      /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    super(
        TalonFX::new, TalonFX::new, CANcoder::new,
        TunerConstants.DrivetrainConstants, configureSwerveChassis());

        CommandScheduler.getInstance().registerSubsystem(this);

        imu = this.getPigeon2();
        //imu.setYaw(0);

        //configureAutoBuilder();

        SmartDashboard.putString("Bot Pose: ", this.getState().Pose.toString());

        System.out.println("*** Configured Drive Subsystem");
  }

  public DriveSubsystem(double OdometryUpdateFrequency) {
    super(
      TalonFX::new, TalonFX::new, CANcoder::new,
      TunerConstants.DrivetrainConstants, OdometryUpdateFrequency, configureSwerveChassis());
    
    CommandScheduler.getInstance().registerSubsystem(this);
    
    imu = this.getPigeon2();
    imu.setYaw(0);

    configureAutoBuilder();
  }

  public static SwerveModuleConstants[] configureSwerveChassis(){
    return new SwerveModuleConstants[] {
      TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD0.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD0.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD0.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD0.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD0.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD0.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD0.isCANCoderIverted()),

      TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD1.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD1.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD1.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD1.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD1.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isCANCoderIverted()),
            
      TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD2.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD2.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD2.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD2.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD2.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isCANCoderIverted()),
      
      TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD3.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD3.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD3.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD3.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD3.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isCANCoderIverted())
    };
  }

   public void setOdometryPoseToSpecificPose(Pose2d p) {
    this.resetPose(p);
  }

  public void stopRobot(){
    drive(0,0,0);
  }

  public Pose2d getPose() {
    //System.out.println("cp: " + this.getState().Pose);
    return this.getState().Pose;
  }


  public void drive(double xV_m_per_s, double yV_m_per_s, double omega_rad_per_s) {
    // this.setControl(
    //   drive.withVelocityX(xV_m_per_s)
    //     .withVelocityY(yV_m_per_s)
    //     .withRotationalRate(omega_rad_per_s)
    // );
    // previousOmegaRotationCommand = omega_rad_per_s/SwerveChassis.MaxAngularRate;
  }

  public double getPitch() {
    return -imu.getPitch().getValueAsDouble();
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) -> setControl(
              m_pathApplyRobotSpeeds.withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the
          // case
          //() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          () -> false,
          this // Subsystem for requirements
      );
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putString("Bot Pose: ", this.getState().Pose.toString());  
    SmartDashboard.putString("Bot Speed: ", this.getState().Speeds.toString());
    //SmartDashboard.putString("Module Position: ", this.getState().ModulePositions.toString());

    System.out.println("TEST\n");
  }
}
