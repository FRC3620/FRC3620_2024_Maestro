package frc.robot.subsystems;

public enum IntakeLocation {
  IntakeIn(0), IntakeOut(7);

  private double intakePositionSetpoint;

  IntakeLocation(double intakePositionsetpoint) {
    this.intakePositionSetpoint = intakePositionsetpoint;
  }

  public double getIntakePositionSetpoint() {
    return intakePositionSetpoint;
  }
}