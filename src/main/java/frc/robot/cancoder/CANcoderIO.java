package frc.robot.cancoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface CANcoderIO {

  public static class CANcoderIOInputs {
    public Rotation2d cancoderPositionRotations = new Rotation2d();
  }

  public void updateInputs(CANcoderIOInputs inputs);
}
