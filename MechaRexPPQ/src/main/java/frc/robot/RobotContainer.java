// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants.TunerConstants;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.PosePrintCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TestSubsystem;

public class RobotContainer {

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static TestSubsystem testSubsystem = new TestSubsystem();

  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  public static XboxController xboxDriveController = new XboxController(5);





  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    

    new JoystickButton(xboxDriveController, 1)
      .onTrue(new PosePrintCommand());
  }

 
  // Alliance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (! alliance.isPresent()) {
        System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
        isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
        System.out.println("*** RED Alliance: "+isAllianceRed);
    }
  }
  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
