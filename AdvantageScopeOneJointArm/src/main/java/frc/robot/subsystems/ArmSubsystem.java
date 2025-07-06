// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    Mechanism2d mechanism = new Mechanism2d(3, 3);
SmartDashboard.putData("MyMechanism", mechanism);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
