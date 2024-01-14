package org.usfirst.frc3620.logger;

import edu.wpi.first.wpilibj.DriverStation;

public class EventLogging {

    // make some levels that correspond to the different SLF4J logging
    // methods. Have the mappings to the underlying j.u.l logging levels.
    public enum Level {
        TRACE(java.util.logging.Level.FINEST), //
        DEBUG(java.util.logging.Level.FINE), //
        INFO(java.util.logging.Level.INFO), //
        WARN(java.util.logging.Level.WARNING), //
        ERROR(java.util.logging.Level.SEVERE);

        java.util.logging.Level julLevel;

        Level(java.util.logging.Level _julLevel) {
            julLevel = _julLevel;
        }
    }

    /**
     * Get an SLF4J logger for a class. Set the underlying j.u.l logger to the
     * desired level.
     * 
     * @param clazz
     *            class for the logger
     * @param l
     *            Level that we want to log at
     * @return
     */
    static public org.slf4j.Logger getLogger(Class<?> clazz, Level l) {
        return getLogger(clazz.getName(), l);
    }

    /**
     * Get an SLF4J logger for a name. Set the underlying j.u.l logger to the
     * desired level.
     * 
     * @param sClazz
     *            name for the logger
     * @param l
     *            Level that we want to log at
     * @return
     */
    static public org.slf4j.Logger getLogger(String sClazz, Level l) {
        //setup();
        // set up the underlying logger to log at the level we want
        //java.util.logging.Logger julLogger = java.util.logging.Logger.getLogger(sClazz);
        //julLogger.setLevel(l.julLevel);

        // and return the SLF4J logger.
        org.slf4j.Logger rv = org.slf4j.LoggerFactory.getLogger(sClazz);
        return rv;
    }
    
    /**
     * Log command starts and stops
     * 
     * @param logger
     * 			  logger to log to.
     */
    public static void commandMessage (org.slf4j.Logger logger) {
  	  Throwable t = new Throwable();
  	  StackTraceElement[] stackTraceElement = t.getStackTrace();
  	  logger.info("command {}", stackTraceElement[1].getMethodName());
    }

    
    /**
     * Write a warning message to the DriverStation.
     * 
     * @param message
     *            Message to log.
     */
    public static final void writeWarningToDS(String message) {
        if (DriverStation.isDSAttached()) {
        	DriverStation.reportWarning(message, false);
        }
    }

    /**
     * Create a String representation of an Exception.
     * 
     * @param t
     * @return
     */
    public static final String exceptionToString(Throwable t) {
        final StackTraceElement[] stackTrace = t.getStackTrace();
        final StringBuilder message = new StringBuilder(1000);
        final String separator = "===\n";
        final Throwable cause = t.getCause();

        message.append("Exception of type ").append(t.getClass().getName());
        message.append("\nMessage: ").append(t.getMessage()).append('\n');
        message.append(separator);
        message.append("   ").append(stackTrace[0]).append('\n');

        for (int i = 1; i < stackTrace.length; i++) {
            message.append(" \t").append(stackTrace[i]).append('\n');
        }

        if (cause != null) {
            final StackTraceElement[] causeTrace = cause.getStackTrace();
            message.append(" \t\tCaused by ")
                    .append(cause.getClass().getName());
            message.append("\n\t\tBecause: ")
                    .append(cause.getMessage());
            message.append("\n\t\t   ").append(causeTrace[0]);
            message.append("\n\t\t\t").append(causeTrace[2]);
            message.append("\n\t\t\t").append(causeTrace[3]);
        }

        return message.toString();
    }

    public static void setup() {
    }

}
