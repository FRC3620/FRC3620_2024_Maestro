import org.junit.Test;

import edu.wpi.first.math.controller.PIDController;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class PIDTest {
    @Test
    public void doNothing() {
        PIDController pid = new PIDController(1, 0, 0);
        pid.enableContinuousInput(-180, 180);

        tryIt (pid, 179, -179);
        /* 
        tryIt (pid, -179, 179);
        tryIt (pid, 0, 181);
        tryIt (pid, 179, 181);
        tryIt (pid, 181, 181);
        */
    }

    void tryIt (PIDController pid, double measurement, double setpoint) {
        double output = pid.calculate(measurement, setpoint);
        System.out.println ("" + measurement + " -> " + output);
    }
}
