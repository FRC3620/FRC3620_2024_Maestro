package frc.robot.subsystems;

public enum IntakeLocation {
  IntakeIn(0), IntakeOut(5.7);

  private double intakePositionSetpoint;

  private IntakeLocation(double intakePositionsetpoint) {
    this.intakePositionSetpoint = intakePositionsetpoint;
  }

  public double getIntakePositionSetpoint() {
    return intakePositionSetpoint;
  }
}