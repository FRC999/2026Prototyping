// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.TunerConstants;
import frc.robot.Constants.SwerveConstants.TunerConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.TunerConstants.SwerveChassis.SwerveModuleConstantsEnum;

import static edu.wpi.first.units.Units.*;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

  private boolean isRobotCentric = false;

  /** Creates a new DriveSubsystem. */

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveChassis.MaxSpeed * SwerveChassis.chassisLinearMoveDeadband)
      .withRotationalDeadband(SwerveChassis.MaxAngularRate * SwerveChassis.chassisAngularMoveDeadband) // Add a 10%
                                                                                                       // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(SwerveChassis.MaxSpeed * SwerveChassis.chassisLinearMoveDeadband)
      .withRotationalDeadband(SwerveChassis.MaxAngularRate * SwerveChassis.chassisAngularMoveDeadband) // Add a 10%
                                                                                                       // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  public DriveSubsystem() {

    super(
        TalonFX::new, TalonFX::new, CANcoder::new,
        TunerConstants.DrivetrainConstants, (SwerveModuleConstants[]) configureSwerveChassis());

    // String name = this.getClass().getSimpleName();
    // name = name.substring(name.lastIndexOf('.') + 1);
    CommandScheduler.getInstance().registerSubsystem(this);

  }

  private double previousOmegaRotationCommand;

  @SuppressWarnings("unchecked")
  public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] configureSwerveChassis() {
    return new SwerveModuleConstants[] {

        // Front Left
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

        // Front Right
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD1.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD1.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD1.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD1.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(-SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD1.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isCANCoderIverted()),

        // Back Left
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD2.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD2.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD2.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD2.getAngleOffset()),
            Meters.of(-SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD2.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isCANCoderIverted()),

        // Back Right
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD3.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD3.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD3.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD3.getAngleOffset()),
            Meters.of(-SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(-SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD3.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isCANCoderIverted())
    };
  }

  public boolean getRobotCentric() {
    return isRobotCentric;
  }

  

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    // System.out.println("X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + "
    // o:" + omega_rad_per_s/SwerveChassis.MaxAngularRate);
    // SmartDashboard.putString("Manual Drive Command Velocities","X: " +
    // xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s);
    this.setControl(
        drive.withVelocityX(xVelocity_m_per_s)
            .withVelocityY(yVelocity_m_per_s)
            .withRotationalRate(omega_rad_per_s));
    previousOmegaRotationCommand = omega_rad_per_s / SwerveChassis.MaxAngularRate;
  }

  public void driveRobotCentric(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    //System.out.println("X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s/SwerveChassis.MaxAngularRate);
    //SmartDashboard.putString("Manual Drive Command Velocities","X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s);
    this.setControl(
      driveRobotCentric.withVelocityX(xVelocity_m_per_s)
        .withVelocityY(yVelocity_m_per_s)
        .withRotationalRate(omega_rad_per_s)
    );
    previousOmegaRotationCommand = omega_rad_per_s / SwerveChassis.MaxAngularRate;
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    drive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
  }

  public Pose2d getPose() {
    return this.getState().Pose;
  }

  public SwerveModuleConstantsEnum getModuleEnum(int modNum){
    switch(modNum){
      case 0: 
        return SwerveModuleConstantsEnum.MOD0;
      case 1: 
        return SwerveModuleConstantsEnum.MOD1;
      case 2:
        return SwerveModuleConstantsEnum.MOD2;
      default:
        return SwerveModuleConstantsEnum.MOD3;
    }
  }

  public void stopRobot(){
    drive(0,0,0);
  }
  //

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("TEST\n");
    // SmartDashboard.putString("T", "T");

    SmartDashboard.putString("Test Bot Pose: ", this.getState().Pose.toString());
  }
}
