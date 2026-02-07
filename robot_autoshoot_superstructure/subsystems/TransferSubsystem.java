
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * TransferSubsystem
 *
 * Purpose:
 * - Meter balls from the spindexer into the turret/shooter with consistent velocity and minimal spin.
 * - "Stage" a ball at the shooter throat (exit of transfer) so the shot can happen immediately when allowed.
 *
 * Recommended sensor layout (your chosen 2-sensor setup):
 * - ENTRY sensor: placed just AFTER the spindexer handoff, inside the transfer tunnel.
 *   This avoids false triggers from spindexer blades.
 * - THROAT sensor: placed at the exit of transfer (right before the shooter/turret throat).
 *
 * Hardware:
 * - One or more motors. Start with one motor; add a second if your transfer needs it.
 */
public class TransferSubsystem extends SubsystemBase {

  private final TalonFX motor;
  private final DutyCycleOut duty = new DutyCycleOut(0.0);

  // Sensors (beam breaks are typical). Wiring convention varies; we invert using constants.
  private final DigitalInput entrySensor;
  private final DigitalInput throatSensor;

  private double commandedDuty = 0.0;

  public TransferSubsystem() {
    motor = new TalonFX(Constants.OperatorConstants.Transfer.MOTOR_ID, Constants.OperatorConstants.Transfer.CANBUS_NAME);
    // TODO: motor inversion/current limits/etc.

    entrySensor = new DigitalInput(Constants.OperatorConstants.Transfer.ENTRY_SENSOR_DIO);
    throatSensor = new DigitalInput(Constants.OperatorConstants.Transfer.THROAT_SENSOR_DIO);
  }

  /** Run transfer at a raw duty cycle in [-1, +1]. */
  public void runDuty(double dutyCycle) {
    commandedDuty = dutyCycle;
    motor.setControl(duty.withOutput(dutyCycle));
  }

  /** Stop transfer. */
  public void stop() {
    runDuty(0.0);
  }

  /** Run transfer at the configured staging speed (keeps ball at throat without hard-feeding). */
  public void runStage() {
    runDuty(Constants.OperatorConstants.Transfer.STAGE_DUTY);
  }

  /** Run transfer at the configured feed speed (inject a ball into the shooter). */
  public void runFeed() {
    runDuty(Constants.OperatorConstants.Transfer.FEED_DUTY);
  }

  /** @return true if a ball is detected at transfer entry (after spindexer). */
  public boolean hasBallAtEntry() {
    boolean raw = entrySensor.get();
    return Constants.OperatorConstants.Transfer.ENTRY_SENSOR_INVERTED ? !raw : raw;
  }

  /** @return true if a ball is detected at the shooter throat (transfer exit). */
  public boolean hasBallAtThroat() {
    boolean raw = throatSensor.get();
    return Constants.OperatorConstants.Transfer.THROAT_SENSOR_INVERTED ? !raw : raw;
  }

  public double getCommandedDuty() {
    return commandedDuty;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Transfer/DutyCmd", commandedDuty);
    SmartDashboard.putBoolean("Transfer/BallAtEntry", hasBallAtEntry());
    SmartDashboard.putBoolean("Transfer/BallAtThroat", hasBallAtThroat());
  }
}
