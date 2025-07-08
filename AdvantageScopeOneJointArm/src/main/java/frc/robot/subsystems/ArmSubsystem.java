// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  Mechanism2d mechanism;
  MechanismRoot2d root;

  TalonFX jointMotor;
  TalonFXSimState jointMotorSim;
  DCMotorSim dcMotorSim;

  TalonFX jointMotorTwo;
  TalonFXSimState jointMotorSimTwo;
  DCMotorSim dcMotorSimTwo;

  MechanismLigament2d arm;
  MechanismLigament2d arm1;
  MechanismLigament2d arm2;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    jointMotor = new TalonFX(1);
    jointMotorSim = jointMotor.getSimState();
    dcMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, ArmConstants.kGearRatio), DCMotor.getKrakenX60(1));

    jointMotorTwo = new TalonFX(2);
    jointMotorSimTwo = jointMotorTwo.getSimState();
    dcMotorSimTwo = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, ArmConstants.kGearRatio), DCMotor.getKrakenX60(1));

    mechanism = new Mechanism2d(4, 6);
    root = mechanism.getRoot("testRoot", 2, 0);
    arm = root.append(new MechanismLigament2d("arm", 1,90, 1, new Color8Bit(255,0,0)));
    arm1 = arm.append(new MechanismLigament2d("arm1", 0.5, 90, 1, new Color8Bit(0,255,0)));
    arm2 = arm1.append(new MechanismLigament2d("arm2", .5, 90, 1, new Color8Bit(0,0,255)));
  }

  public void simulationInit() {
    jointMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    jointMotorSimTwo.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void simulationPeriodic() {
    jointMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    jointMotorSimTwo.setSupplyVoltage(RobotController.getBatteryVoltage());


    dcMotorSim.setInput(jointMotorSim.getMotorVoltage());
    dcMotorSim.update(0.020);
    dcMotorSimTwo.setInput(jointMotorSimTwo.getMotorVoltage());
    dcMotorSimTwo.update(0.020);

    jointMotorSim.setRawRotorPosition(dcMotorSim.getAngularPosition().times(ArmConstants.kGearRatio));
    jointMotorSim.setRotorVelocity(dcMotorSim.getAngularVelocity().times(ArmConstants.kGearRatio));
    jointMotorSimTwo.setRawRotorPosition(dcMotorSimTwo.getAngularPosition().times(ArmConstants.kGearRatio));
    jointMotorSimTwo.setRotorVelocity(dcMotorSimTwo.getAngularVelocity().times(ArmConstants.kGearRatio));

    arm1.setAngle(dcMotorSim.getAngularPositionRad()*(180./Math.PI)*(1/ArmConstants.kGearRatio));
    arm2.setAngle(dcMotorSimTwo.getAngularPositionRad()*(180./Math.PI)*(1/ArmConstants.kGearRatio));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    jointMotor.set(RobotContainer.m_driverController.getLeftX());
    jointMotorTwo.set(RobotContainer.m_driverController.getRightX());

    SmartDashboard.putData("MyMechanism", mechanism);
  }
}
