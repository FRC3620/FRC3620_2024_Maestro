package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CSPMXSW;
import org.usfirst.frc3620.misc.FakeMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FakeSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
    FakeMotor m1, m2, m3, m5;
    CSPMXSW m4;
    boolean m5_added = false;
    public FakeSubsystem(String name, int canSparkMaxNumber) {
      super(name);
      m1 = new FakeMotor(1);
      m2 = new FakeMotor(2);
      m3 = new FakeMotor(canSparkMaxNumber);
      CANSparkMax realCANSparkMax = new CANSparkMax(canSparkMaxNumber, MotorType.kBrushless);
      m4 = new CSPMXSW(realCANSparkMax);
      logger.info("{} has CANSparkMAX {}", name, realCANSparkMax.getDeviceId());
      addChild("foo", m1);
      addChild("bar", m2);
      addChild("baz" + canSparkMaxNumber, m3);
      addChild("CAN" + canSparkMaxNumber, m4);
    }

    @Override
    public void periodic() {
      if (m5 == null) {
        m5 = new FakeMotor(5);
        addChild("latebloomer", m5);
      }

    }
  }
