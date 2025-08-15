// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.RobotContainer;

public class DriveManuallyCommand extends Command {
  private final DoubleSupplier mVxSupplier;
	private final DoubleSupplier mVySupplier;
	private final DoubleSupplier mOmegaSupplier;

  /** Creates a new DriveManuallyCommand. */
  public DriveManuallyCommand(DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);

    mVxSupplier = vxSupplier;
		mVySupplier = vySupplier;
		mOmegaSupplier = omegaSupplier;
  }

  /**
   * This method man be used when troubleshooting controller inputs
   * @param dx
   * @param dy
   * @param dm
   */
  @SuppressWarnings("unused")
   private void driveControlTelemetry(double dx, double dy, double dm){
    System.out.print("DX "+ dx);
    System.out.print(" DY "+ dy);
    System.out.println(" Dm "+ dm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = mVxSupplier.getAsDouble()
    //alexand
    *0.8
    ;
		double yInput = mVySupplier.getAsDouble()
    //alexand
    *0.8
    ;
		double omegaInput = mOmegaSupplier.getAsDouble();

    // SmartDashboard.putString("Joystick Values: ", "X: " + xInput + " Y: " + yInput + " Z: " + omegaInput);

    // if(RobotContainer.isAllianceRed && RobotContainer.isReversingControllerAndIMUForRed){
    //   xInput = -xInput;
    //   yInput = -yInput;
    // }
    
    if(!  RobotContainer.driveSubsystem.getRobotCentric()) {

    RobotContainer.driveSubsystem.drive(
      xInput * SwerveConstants.MaxSpeed, 
     //0,
      yInput * SwerveConstants.MaxSpeed,
      //0
      omegaInput * SwerveConstants.MaxAngularRate
    );
    } else {
      RobotContainer.driveSubsystem.driveRobotCentric(
        xInput * SwerveConstants.MaxSpeed,
       //0,
        yInput * SwerveConstants.MaxSpeed,
        //0
        omegaInput * SwerveConstants.MaxAngularRate
  
      );
    } 
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
