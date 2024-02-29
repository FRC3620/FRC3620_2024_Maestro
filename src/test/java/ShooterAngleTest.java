import static org.junit.Assert.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.Test;

import frc.robot.ShooterCalcutlaiter;
import frc.robot.subsystems.ShooterSpeedAndAngle;

/**
 * Add your docs here.
 */
public class ShooterAngleTest {

  @Test
  public void test01() {
    testOneSet(3, 63.0000000000);
    testOneSet(3.25, 60.2734375000);
    testOneSet(3.5, 57.7187500000);
    testOneSet(3.75,55.3359375000);
    testOneSet(4, 53.1250000000);
    testOneSet(4.25, 51.0859375000);
    testOneSet(4.5, 49.2187500000);
    testOneSet(4.75, 47.5234375000);
    testOneSet(5, 46.0000000000);
    testOneSet(5.25, 44.6484375000);
    testOneSet(5.5, 43.4687500000);
    testOneSet(5.75, 42.4609375000);
    testOneSet(6, 41.6250000000);
    testOneSet(6.25, 40.9609375000);
    testOneSet(6.5, 40.4687500000);
    testOneSet(6.75, 40.1484375000);
    testOneSet(7, 40.00);
    testOneSet(7.25, 39.62);
    testOneSet(7.5, 39.25);
    testOneSet(7.75, 38.87);
    testOneSet(8, 38.50);
    testOneSet(8.25, 38.12);
    testOneSet(8.5, 37.75);
    testOneSet(8.75, 37.37);
    testOneSet(9, 37.00);
    testOneSet(9.25, 36.62);
    testOneSet(9.5, 36.25);
    testOneSet(9.75, 35.87);
    testOneSet(10, 35.50);
    testOneSet(10.25, 35.12);
    testOneSet(10.5, 34.75);
    testOneSet(10.75, 34.37);
    testOneSet(11, 34.00);
    testOneSet(11.25,33.8125);
    testOneSet(11.5,33.6250);
    testOneSet(11.75,33.4375);
    testOneSet(12,33.2500);
    testOneSet(12.25,33.0625);
    testOneSet(12.5,32.8750);
    testOneSet(12.75,32.6875);
    testOneSet(13,32.5000);
    testOneSet(13.25,32.3125);
    testOneSet(13.5,32.1250);
    testOneSet(13.75,31.9375);
    testOneSet(14,31.7500);
    testOneSet(14.25,31.5625);
    testOneSet(14.5,31.3750);
    testOneSet(14.75,31.1875);
    testOneSet(15,31.0000);
  }

  void testOneSet (double distance, double expectedAngle) {
    ShooterSpeedAndAngle sa = ShooterCalcutlaiter.CalculaiteAngleFt(distance);
    double calculatedAngle = sa.getPosition();
    assertEquals(expectedAngle, calculatedAngle, 0.1, "Calculated angle");
  }

}