package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * SIM-only holder for shooter pitch (hood) angle.
 * Publishes to NT so AdvantageScope / dashboards can display it.
 */
public class ShooterHoodSim {
  private double pitchDeg = 0.0;

  private final DoublePublisher pitchPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("Shooter/PitchDeg")
          .publish();

  public void setPitchDeg(double pitchDeg) {
    this.pitchDeg = pitchDeg;
    pitchPub.set(pitchDeg);
  }

  public double getPitchDeg() {
    return pitchDeg;
  }
}
