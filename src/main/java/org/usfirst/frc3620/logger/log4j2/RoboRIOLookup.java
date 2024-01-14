package org.usfirst.frc3620.logger.log4j2;

import java.util.Date;

import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.LogEvent;
import org.apache.logging.log4j.core.config.plugins.Plugin;
import org.apache.logging.log4j.core.lookup.StrLookup;
import org.apache.logging.log4j.status.StatusLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

@Plugin(name = "roborio", category = StrLookup.CATEGORY)
public class RoboRIOLookup implements StrLookup {
    protected final static Logger logger = StatusLogger.getLogger();

    String logdir = null;
    String timestamp = null;

    public RoboRIOLookup() {
        super();
    }

    @Override
    public String lookup(String key) {
        if ("logdir".equalsIgnoreCase(key)) {
            if (logdir == null) {
                logdir = LoggingMaster.getLoggingDirectory().getAbsolutePath();
                logger.debug ("logging to directory {}", logdir);
            }
            return logdir;
        } else if ("timestamp".equalsIgnoreCase(key)) {
            if (timestamp == null) {
                Date ts = LoggingMaster.getTimestamp();
                if (ts != null) {
                    timestamp = LoggingMaster.convertTimestampToString(ts);
                    logger.debug ("logging timestamp {}", timestamp);
                }
            }
            return timestamp;
        }
        return null;
    }

    @Override
    public String lookup(LogEvent event, String key) {
        return lookup(key);
    }
}
