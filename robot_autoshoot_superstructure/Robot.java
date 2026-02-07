// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.LogFileUtil;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // --- AdvantageKit setup (NO robot behavior changes) ---
    Logger.recordMetadata("Project", "2026Competition");
    Logger.recordMetadata("Mode", "Competition");

    if (isReal()) {
      // Log to roboRIO internal storage (NO USB required)
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));

      // Allow live viewing in AdvantageScope
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Simulation / replay mode
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(logPath + "_sim"));
    }

    Logger.start();
    m_robotContainer = new RobotContainer();
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void robotInit() {
    RobotContainer.setIfAllianceRed();

    // The YAW should be set by autos and not really here
    //RobotContainer.driveSubsystem.zeroYaw(); //Sets Yaw to 180 if on Red Alliance, or 0 on Blue (theoretically)
    // RobotContainer.driveSubsystem.zeroYawInitial();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.setIfAllianceRed();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
