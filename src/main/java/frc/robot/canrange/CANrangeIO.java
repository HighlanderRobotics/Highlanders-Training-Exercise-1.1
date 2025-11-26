package frc.robot.canrange;

public interface CANrangeIO {

  public static class CANrangeIOInputs {
    public double distanceMeters = 0.0;
    public boolean isDetected = false;
  }

  public void updateInputs(CANrangeIOInputs inputs);
}
