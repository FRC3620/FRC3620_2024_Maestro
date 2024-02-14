package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.HashMap;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.logger.DataLoggerPrelude;

import edu.wpi.first.wpilibj.RobotController;

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
            dataLogger.addDataProvider ("mem.gc." + smooshedName + ".rate", () -> gcRates.get(name).rate);
            dataLogger.addDataProvider ("mem.gc." + smooshedName + ".time", () -> gcTimes.get(name));
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
            long t0 = RobotController.getFPGATime(); // microseconds
            long c = gc.getCollectionCount();
            long t = gc.getCollectionTime(); // milliseconds
            gcCounts.put(name, gc.getCollectionCount());
            gcTimes.put(name, gc.getCollectionTime());
        }
    }

    class DeltaOverTime {
        Long t0;
        Long v0;
        String rate = "";

        void update(long t1, long v1) {
            if (t0 != null && v0 != null) {
                double d = v1 - v0;
                rate = DataLogger.f2((d == 0) ? 0 : (t1 - t0) / d);
            }
            t0 = t1;
            v0 = v1;
        }
    }
}
