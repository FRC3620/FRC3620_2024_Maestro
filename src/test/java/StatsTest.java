import java.util.Random;
import java.util.random.RandomGenerator;

import org.junit.Test;
import org.usfirst.frc3620.misc.Utilities.SlidingWindowStats;

public class StatsTest {
    @Test
    public void t0() {
        SlidingWindowStats stats = new SlidingWindowStats(100);
        for (int i = 0; i < 100; i++) stats.addValue(10.0);
        System.out.println(stats);
        for (int i = 0; i < 100; i++) stats.addValue(10.0);
        System.out.println(stats);
    }

    @Test
    public void t1() {
        Random r = new Random();
        SlidingWindowStats stats = new SlidingWindowStats(100);
        for (int i = 0; i < 100; i++) stats.addValue(r.nextGaussian());
        System.out.println(stats);
        for (int i = 0; i < 100; i++) stats.addValue(r.nextGaussian());
        System.out.println(stats);
        stats.addValue(10);
        stats.addValue(-10);
        System.out.println(stats);
    }
}
