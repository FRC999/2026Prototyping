// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveManuallyCommand extends Command {
  private final DoubleSupplier mVxSupplier;
	private final DoubleSupplier mVySupplier;
	private final DoubleSupplier mOmegaSupplier;
  /** Creates a new DriveManuallyCommand. */
  public DriveManuallyCommand(DoubleSupplier vxSupplier,  DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
    mVxSupplier = vxSupplier;
		mVySupplier = vySupplier;
		mOmegaSupplier = omegaSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    // double xInput = mVxSupplier.getAsDouble();
		// double yInput = mVySupplier.getAsDouble();
		// double omegaInput = mOmegaSupplier.getAsDouble();

    // //System.out.println("X: " + xInput + " Y: " + yInput + " Z: " + omegaInput);


    // RobotContainer.driveSubsystem.drive(
    //   xInput * SwerveChassis.MaxSpeed,
    //  //0,
    //   yInput * SwerveChassis.MaxSpeed,
    //   //0
    //   omegaInput * SwerveChassis.MaxAngularRate
    //);
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
