package org.usfirst.frc3620.misc;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
@SuppressWarnings("removal")
public class MotorStatus {

    String name;
    double requestedRPM = -1;
    double requestedSensorVelocity = -1;
    double actualRPM = -1;
    double actualSensorVelocity = -1;
    double statorCurrent = -1;
    double supplyCurrent = -1;
    double appliedPower = -1;

    public MotorStatus(String _name) {
      this.name = _name;
    }

    public String getName() {
      return name;
    }

    public double getRequestedRPM() {
      return requestedRPM;
    }

    public double getRequestedSensorVelocity() {
      return requestedSensorVelocity;
    }

    public double getActualSensorVelocity() {
      return actualSensorVelocity;
    }

    public double getActualRPM() {
      return actualRPM;
    }

    public double getStatorCurrent() {
      return statorCurrent;
    }

    public double getSupplyCurrent() {
      return supplyCurrent;
    }

    public double getAppliedPower() {
      return appliedPower;
    }

    public void setRequestedRPM(double r){
        requestedRPM = r;
        SmartDashboard.putNumber(name + ".rpm.target", r);
    }

    public void setRequestedSensorVelocity(double v) {
        requestedSensorVelocity = v;
        SmartDashboard.putNumber(name + ".velocity.target", v);
    }

  public void gatherActuals(TalonFX m, String prefix) {
    if (m != null) {
      actualSensorVelocity = m.getVelocity().getValue();
      actualRPM = actualSensorVelocity * 600 / 2048;
      statorCurrent = m.getStatorCurrent().getValue();
      supplyCurrent = m.getSupplyCurrent().getValue();
      appliedPower = m.getDutyCycle().getValue() / 100.0;
    } else {
      actualSensorVelocity = -1;
      actualRPM = -1;
      statorCurrent = -1;
      supplyCurrent = -1;
      appliedPower = -1;
    }
    updateDashboard(prefix);
  }

  void updateDashboard(String prefix) {
    SmartDashboard.putNumber(prefix + ".velocity.actual", actualSensorVelocity);
    SmartDashboard.putNumber(prefix + ".rpm.actual", actualRPM);
    SmartDashboard.putNumber(prefix + ".current.stator", statorCurrent);
    SmartDashboard.putNumber(prefix + ".current.supply", supplyCurrent);
    SmartDashboard.putNumber(prefix + ".applied.power", appliedPower);
  }

  public void gatherActuals(CANSparkMax m, String prefix) {
    if (m != null) {
      RelativeEncoder encoder = m.getEncoder();
      actualSensorVelocity = encoder.getVelocity();
      actualRPM = actualSensorVelocity;
      statorCurrent = m.getOutputCurrent();
      supplyCurrent = -1;
      appliedPower = m.getAppliedOutput();
    } else {
      actualSensorVelocity = -1;
      actualRPM = -1;
      statorCurrent = -1;
      supplyCurrent = -1;
      appliedPower = -1;
    }
    updateDashboard(prefix);
  }
}