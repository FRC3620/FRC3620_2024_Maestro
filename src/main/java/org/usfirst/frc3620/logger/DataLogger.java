package org.usfirst.frc3620.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

public class DataLogger extends DataLoggerBase {
    double flushInterval = 2.0;

    WriterThingy writerThingy = new WriterThingy();
    Timer timer = null;

    public void setFlushInterval(double flushInterval) {
        this.flushInterval = flushInterval;
    }

    @Override
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
        timer.schedule(new SlowDataLoggerTimerTask(writerThingy), 0, interval);
    }

    @Override
    public void setInterval(double s) {
        double oldInterval = getInterval();
        super.setInterval(s);
        if (s != oldInterval) {
            if (timer != null) {
                schedule();
            }
        }
    }

    class SlowDataLoggerTimerTask extends TimerTask {
        WriterThingy writerThingy;

        SlowDataLoggerTimerTask(WriterThingy writerThingy) {
            this.writerThingy = writerThingy;
        }

        @Override
        public void run() {
            this.writerThingy.run();
        }
    }

    class WriterThingy {
        SimpleDateFormat format = new SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS");
        double tFlushed = getTimeInSeconds();
        double t0 = getTimeInSeconds();
        PrintWriter w;
        Object[] data;

        void run() {
            if (w == null) {
                synchronized (DataLogger.this) {
                    if (w == null) {
                        File outputFile = setupOutputFile();
                        if (outputFile != null) {
                            try {
                                w = new PrintWriter(new FileWriter(outputFile));
                                logger.info("Writing dataLogger to {}", outputFile.getAbsolutePath());

                                writeHeader(w, namedDataProviders, metadata);
                                w.flush();

                            } catch (IOException e) {
                                logger.error("trouble when logging data: {}", e);
                            }
                        }
                    }
                }
            }
            if (w != null) {
                if (data == null) {
                    data = new Object[namedDataProviders.size()];
                }

                for (var prelude : preludes) {
                    prelude.dataLoggerPrelude();
                }

                for (int i = 0; i < data.length; i++) {
                    NamedDataProvider namedDataProvider = namedDataProviders.get(i);
                    try {
                        data[i] = namedDataProvider.iDataLoggerDataProvider.get();
                    } catch (Exception e) {
                        data[i] = e.toString(); // "ERROR";
                    }
                }

                for (var postlude : postludes) {
                    postlude.dataLoggerPostlude();
                }

                double t = getTimeInSeconds();
                Date curDate = new Date();

                w.print(','); // 1st column is metadata
                w.print(format.format(curDate));
                w.print(',');
                w.format("%.6f", t - t0);

                for (Object v : data) {
                    w.print(',');
                    w.print(v);
                }
                w.println();

                // flush once every couple seconds
                if (t - tFlushed > flushInterval)
                    w.flush();
            }
        }
    }

    private static final ThreadLocal<DecimalFormat> f2formatter = new ThreadLocal<DecimalFormat>() {
        @Override
        protected DecimalFormat initialValue() {
            return new DecimalFormat("#.##");
        }
    };

    public static String f2(double value) {
        return f2formatter.get().format(value);
    }
}
