package org.usfirst.frc3620.logger;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.BooleanSupplier;

import org.slf4j.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.RobotController;

public class WPIDataLogger {
  Logger logger = EventLogging.getLogger(getClass());

  double flushInterval = 2.0;
  boolean started /* = false */;

  double intervalInSeconds = 0.1;

  Timer timer = null;

  List<DataLoggerPrelude> preludes = new ArrayList<>();
  List<DataLoggerPostlude> postludes = new ArrayList<>();

  List<IDataProvider> providers = new ArrayList<>();

  TimeCollector timeCollector;

  private static final ThreadLocal<SimpleDateFormat> dateFormatter = new ThreadLocal<SimpleDateFormat>() {
    @Override
    protected SimpleDateFormat initialValue() {
      return new SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS");
    }
  };

  static String formatDate(Date value) {
    return dateFormatter.get().format(value);
  }

  class TimeCollector {
    Date curDate;

    public void update(long milliseconds) {
      curDate = new Date(milliseconds);
    }

    public String getTimeString() {
      return formatDate(curDate);
    }
  }

  public WPIDataLogger() {
    timeCollector = new TimeCollector();
    addStringDataProvider("time", () -> timeCollector.getTimeString());
  }

  public void start() {
    started = true;

    startTimer();
  }

  public void setFlushInterval(double flushInterval) {
    this.flushInterval = flushInterval;
  }

  public void startTimer() {
    schedule();
  }

  private void schedule() {
    if (timer != null) {
      logger.info("Cancelling {}", timer);
      timer.cancel();
    }

    timer = new Timer();
    long interval = Math.max(1, Math.round(getInterval() * 1000));
    logger.info("New timer {}, interval = {}ms", timer, interval);
    timer.schedule(new WriterThingy(), 0, interval);
  }

  public void setInterval(double s) {
    double oldInterval = getInterval();
    intervalInSeconds = s;
    if (s != oldInterval) {
      if (timer != null) {
        schedule();
      }
    }
  }

  public double getInterval() {
    return intervalInSeconds;
  }

  class WriterThingy extends TimerTask {
    double tFlushed = 0;
    DataLog dataLog;
    Set<IDataProvider> wentBoom = new HashSet<>();

    @Override
    public void run() {
      if (dataLog == null) {
        synchronized (WPIDataLogger.this) {
          if (dataLog == null) {
            String outputFilename = getOutputFilename();
            if (outputFilename != null) {
              dataLog = new DataLog(LoggingMaster.getLoggingDirectory().getAbsolutePath(), outputFilename);
              logger.info("Writing dataLogger to {}", outputFilename);
            }

            for (var dataProvider : providers) {
              dataProvider.connectToDataLog(dataLog);
            }

            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            int m_ntEntryLogger = inst.startEntryDataLog(dataLog, "/FMSInfo/", "NT:/FMSInfo/");
            logger.info ("ntEntryLogger = {}", m_ntEntryLogger);
          }
        }
      }
      if (dataLog != null) {
        long timestamp = RobotController.getFPGATime(); // time in microseconds
        double t = timestamp / 1000000.0; // time in seconds
        timeCollector.update(System.currentTimeMillis());

        for (var prelude : preludes) {
          prelude.dataLoggerPrelude();
        }

        for (var dataProvider : providers) {
          try {
            dataProvider.updateWithTimestamp(timestamp);
          } catch (Exception ex) {
            if (!wentBoom.contains(dataProvider)) {
              logger.error("dataprovider {} threw exception", dataProvider.getName(), ex);
              wentBoom.add(dataProvider);
            }
          }
        }

        for (var postlude : postludes) {
          postlude.dataLoggerPostlude();
        }

        // flush once every couple seconds
        if (t - tFlushed > flushInterval) {
          dataLog.flush();
          tFlushed = t;
        }
      }
    }
  }

  public void addPrelude(DataLoggerPrelude prelude) {
    if (!started) {
      preludes.add(prelude);
    } else {
      logger.error("Cannot addPrelude(...) after start()");
    }
  }

  public void addPostlude(DataLoggerPostlude postlude) {
    if (!started) {
      postludes.add(postlude);
    } else {
      logger.error("Cannot addPostlude(...) after start()");
    }
  }

  String outputFilename;

  String getOutputFilename() {
    if (outputFilename == null) {
      synchronized (this) {
        if (outputFilename == null) {
          Date timestamp = LoggingMaster.getTimestamp();
          if (timestamp != null) {
            String _timestampString = LoggingMaster.convertTimestampToString(timestamp);
            outputFilename = _timestampString + ".wpilog";
            logger.info("getOutputFilename is {}", outputFilename);
          }
        }
      }
    }
    return outputFilename;
  }

  public IDataProvider addDataProvider(IDataProvider provider) {
    providers.add(provider);
    return provider;
  }

  public BooleanDataProvider addBooleanDataProvider(String name, BooleanSupplier supplier) {
    BooleanDataProvider rv = new BooleanDataProvider(name, supplier);
    providers.add(rv);
    return rv;
  }

  public LongDataProvider addLongDataProvider(String name, LongSupplier supplier) {
    LongDataProvider rv = new LongDataProvider(name, supplier);
    providers.add(rv);
    return rv;
  }

  public DoubleDataProvider addDoubleDataProvider(String name, DoubleSupplier supplier) {
    DoubleDataProvider rv = new DoubleDataProvider(name, supplier);
    providers.add(rv);
    return rv;
  }

  public StringDataProvider addStringDataProvider(String name, Supplier<String> supplier) {
    StringDataProvider rv = new StringDataProvider(name, supplier);
    providers.add(rv);
    return rv;
  }

  public Pose2dDataProvider addPose2dDataProvider(String name, Supplier<Pose2d> supplier) {
    Pose2dDataProvider rv = new Pose2dDataProvider(name, supplier);
    providers.add(rv);
    return rv;
  }

  public Pose3dDataProvider addPose3dDataProvider(String name, Supplier<Pose3d> supplier) {
    Pose3dDataProvider rv = new Pose3dDataProvider(name, supplier);
    providers.add(rv);
    return rv;
  }

  interface IDataProvider {

    String getName();

    void connectToDataLog(DataLog datalog);

    void update();

    void updateWithTimestamp(long timestamp);
  }

  abstract class PrimitiveDataProvider implements IDataProvider {
    String name;

    PrimitiveDataProvider(String name) {
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }

    @Override
    public abstract void connectToDataLog(DataLog dataLog);

    @Override
    public abstract void update();

    @Override
    public abstract void updateWithTimestamp(long timestamp);
  }

  public class DoubleDataProvider extends PrimitiveDataProvider {
    DoubleSupplier supplier;
    DoubleLogEntry logEntry;

    DoubleDataProvider(String name, DoubleSupplier supplier) {
      super(name);
      this.supplier = supplier;
    }

    @Override
    public void connectToDataLog(DataLog dataLog) {
      logEntry = new DoubleLogEntry(dataLog, this.name);
    }

    @Override
    public void update() {
      logEntry.append(supplier.getAsDouble());
    }

    @Override
    public void updateWithTimestamp(long timestamp) {
      logEntry.append(supplier.getAsDouble(), timestamp);
    }
  }

  public class LongDataProvider extends PrimitiveDataProvider {
    LongSupplier supplier;
    IntegerLogEntry logEntry;

    LongDataProvider(String name, LongSupplier supplier) {
      super(name);
      this.supplier = supplier;
    }

    @Override
    public void connectToDataLog(DataLog dataLog) {
      logEntry = new IntegerLogEntry(dataLog, name);
    }

    @Override
    public void update() {
      logEntry.append(supplier.getAsLong());
    }

    @Override
    public void updateWithTimestamp(long timestamp) {
      logEntry.append(supplier.getAsLong(), timestamp);
    }
  }

  public class BooleanDataProvider extends PrimitiveDataProvider {
    BooleanSupplier supplier;
    BooleanLogEntry logEntry;

    BooleanDataProvider(String name, BooleanSupplier supplier) {
      super(name);
      this.supplier = supplier;
    }

    @Override
    public void connectToDataLog(DataLog dataLog) {
      logEntry = new BooleanLogEntry(dataLog, name);
    }

    @Override
    public void update() {
      logEntry.append(supplier.getAsBoolean());
    }

    @Override
    public void updateWithTimestamp(long timestamp) {
      logEntry.append(supplier.getAsBoolean(), timestamp);
    }
  }

  public class StringDataProvider extends PrimitiveDataProvider {
    Supplier<String> supplier;
    StringLogEntry dataLogEntry;

    StringDataProvider(String name, Supplier<String> supplier) {
      super(name);
      this.supplier = supplier;
    }

    @Override
    public void connectToDataLog(DataLog dataLog) {
      dataLogEntry = new StringLogEntry(dataLog, name);
    }

    @Override
    public void update() {
      dataLogEntry.append(supplier.get());
    }

    @Override
    public void updateWithTimestamp(long timestamp) {
      dataLogEntry.append(supplier.get(), timestamp);
    }

    public void update(String v) {
      dataLogEntry.append(v);
    }

    public void updateWithTimestamp(String v, long timestamp) {
      dataLogEntry.append(v, timestamp);
    }
  }

  abstract class DataProvider<T extends Object> implements IDataProvider {
    String name;
    Supplier<T> supplier;
    DataLogEntry dataLogEntry;

    DataProvider(String name, Supplier<T> supplier) {
      this.name = name;
      this.supplier = supplier;
    }

    @Override
    public String getName() {
      return name;
    }

    @Override
    public abstract void connectToDataLog(DataLog dataLog);

    @Override
    public abstract void update();

    @Override
    public abstract void updateWithTimestamp(long timestamp);

    public abstract void update(T v);

    public abstract void updateWithTimestamp(T v, long timestamp);
  }

  class StructDataProvider<T> extends DataProvider<T> {
    Struct<T> struct;
    StructLogEntry<T> logEntry;

    StructDataProvider(String name, Supplier<T> supplier, Struct<T> struct) {
      super(name, supplier);
      this.struct = struct;
    }

    @Override
    public void connectToDataLog(DataLog dataLog) {
      logEntry = StructLogEntry.create(dataLog, name, struct);
    }

    @Override
    public void update() {
      logEntry.append(supplier.get());
    }

    @Override
    public void updateWithTimestamp(long timestamp) {
      logEntry.append(supplier.get(), timestamp);
    }

    @Override
    public void update(T v) {
      logEntry.append(v);
    }

    @Override
    public void updateWithTimestamp(T v, long timestamp) {
      logEntry.append(v, timestamp);
    }
  }

  public class Pose2dDataProvider extends StructDataProvider<Pose2d> {
    Pose2dDataProvider(String name, Supplier<Pose2d> supplier) {
      super(name, supplier, Pose2d.struct);
    }
  }

  public class Pose3dDataProvider extends StructDataProvider<Pose3d> {
    Pose3dDataProvider(String name, Supplier<Pose3d> supplier) {
      super(name, supplier, Pose3d.struct);
    }
  }

}
