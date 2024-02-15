package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.HashMap;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.logger.DataLoggerPrelude;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class MemoryDataLogger implements DataLoggerPrelude {
    Runtime runtime;

    static final double M = 1024 * 1024;

    long free, total, max;
    long t0;

    HashMap<String,Long> gcCounts = new HashMap<>();
    HashMap<String,Long> gcTimes = new HashMap<>();
    HashMap<String,DeltaOverTime> gcRates = new HashMap<>();
    HashMap<String,DeltaOverTime> gcCPUFraction = new HashMap<>();

    /*
     * see https://stackoverflow.com/a/467366/17887564 for infomation of the GC stats
     */

    public MemoryDataLogger (DataLogger dataLogger) {
        dataLogger.addPrelude(this);
    
		runtime = Runtime.getRuntime();

		dataLogger.addDataProvider ("mem.used", () -> DataLogger.f2((total - free) / M));
		dataLogger.addDataProvider ("mem.total", () -> DataLogger.f2(total / M));
		dataLogger.addDataProvider ("mem.max", () -> DataLogger.f2(max / M));

        for (GarbageCollectorMXBean gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            String name = gc.getName();
            String smooshedName = name.replace(" ", "");
            dataLogger.addDataProvider ("mem.gc." + smooshedName + ".count", () -> gcCounts.get(name));
            gcRates.put(name, new DeltaOverTime());
            dataLogger.addDataProvider ("mem.gc." + smooshedName + ".rate", () -> gcRates.get(name).rate);
            dataLogger.addDataProvider ("mem.gc." + smooshedName + ".time", () -> gcTimes.get(name));
            gcCPUFraction.put(name, new DeltaOverTime());
            dataLogger.addDataProvider ("mem.gc." + smooshedName + ".cpu", () -> gcCPUFraction.get(name).rate);
        }
    }

    @Override
    public void dataLoggerPrelude() {
        free = runtime.freeMemory();
        total = runtime.totalMemory();
        max = runtime.maxMemory();

        for (GarbageCollectorMXBean gc : ManagementFactory.getGarbageCollectorMXBeans()) {
            String name = gc.getName();
            double t0 = Timer.getFPGATimestamp(); // seconds
            long c = gc.getCollectionCount();
            long t = gc.getCollectionTime(); // milliseconds
            gcCounts.put(name, gc.getCollectionCount());
            gcTimes.put(name, gc.getCollectionTime());
            gcRates.get(name).update(t0, c);
            gcCPUFraction.get(name).update(t0, t / 10.0); // units -> %, milliseconds -> seconds
        }
    }

    class DeltaOverTime {
        Double t0;
        Double v0;
        String rate = "";

        void update(double t1, double v1) {
            if (t0 != null) {
                double tDelta = t1 - t0;
                rate = DataLogger.f2((tDelta == 0) ? 0 : (v1 - v0) / tDelta);
            }
            t0 = t1;
            v0 = v1;
        }
    }
}
