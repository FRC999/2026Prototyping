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
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX,TalonFX,CANcoder> implements Subsystem {
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

        super(
          TalonFX::new, TalonFX::new, CANcoder::new,
          new SwerveDrivetrainConstants()
            .withPigeon2Id(15), 
          //new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>(
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration,CANcoderConfiguration>().
              createModuleConstants(
                2, 1, 20, Rotations.of(0), Inches.of(20), Inches.of(20), false, true, false
              ).withDriveMotorGearRatio(6.12)
              .withSteerMotorGearRatio(21.43)
              .withCouplingGearRatio(3.57)
              ,
              new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration,CANcoderConfiguration>().
              createModuleConstants(
                4, 3, 20, Rotations.of(0), Inches.of(-20), Inches.of(-20), false, true, false
              ).withDriveMotorGearRatio(6.12)
              .withSteerMotorGearRatio(21.43)
              .withCouplingGearRatio(3.57)
          //)
          );

          //String name = this.getClass().getSimpleName();
          //name = name.substring(name.lastIndexOf('.') + 1);
          CommandScheduler.getInstance().registerSubsystem(this);

    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    System.out.println("TEST\n");
    SmartDashboard.putString("T", "T");
  }
}
