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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

   Mechanism2d mechanism;
  MechanismRoot2d root;

  TalonFX jointMotor;
  TalonFXSimState jointMotorSim;
  SingleJointedArmSim armSim;

  MechanismLigament2d arm;
  MechanismLigament2d arm1;

  public ArmSubsystem() {
    jointMotor = new TalonFX(1);
    jointMotorSim = jointMotor.getSimState();
    armSim = new SingleJointedArmSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
        DCMotor.getKrakenX60(1), ArmConstants.kGearRatio, 0.5,0, Math.PI*2, true, 0);
    armSim.estimateMOI(0.5, 100);


    mechanism = new Mechanism2d(4, 6);
    root = mechanism.getRoot("testRoot", 2, 0);
    arm = root.append(new MechanismLigament2d("arm", 1,90, 1, new Color8Bit(255,0,0)));
    arm1 = arm.append(new MechanismLigament2d("arm1", 0.5, 90, 1, new Color8Bit(0,255,0)));
  }
  
  public void simulationInit() {
    jointMotorSim.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void simulationPeriodic() {

    armSim.setInput(jointMotorSim.getMotorVoltage());
    armSim.update(0.020);

    jointMotorSim.setRawRotorPosition(armSim.getAngleRads()*ArmConstants.kGearRatio);
    jointMotorSim.setRotorVelocity(armSim.getVelocityRadPerSec()*ArmConstants.kGearRatio);

    arm1.setAngle(armSim.getAngleRads()*(180./Math.PI)*(1/ArmConstants.kGearRatio));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    jointMotor.set(RobotContainer.m_driverController.getLeftX());

    SmartDashboard.putData("MyMechanism", mechanism);
  }
}

