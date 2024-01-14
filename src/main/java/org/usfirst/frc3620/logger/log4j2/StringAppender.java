package org.usfirst.frc3620.logger.log4j2;

import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.core.*;
import org.apache.logging.log4j.core.appender.*;
import org.apache.logging.log4j.core.config.Configuration;
import org.apache.logging.log4j.core.config.LoggerConfig;
import org.apache.logging.log4j.core.config.plugins.Plugin;
import org.apache.logging.log4j.core.config.plugins.PluginAttribute;
import org.apache.logging.log4j.core.config.plugins.PluginElement;
import org.apache.logging.log4j.core.config.plugins.PluginFactory;
import org.apache.logging.log4j.core.layout.PatternLayout;

import java.io.*;

/**
 * A Custom Appender for Log4j2 that logs to a String. This is useful for testing logging.
 *
 * @author rewolf
 */
@Plugin(name = "StringAppender", category = Core.CATEGORY_NAME, elementType = Appender.ELEMENT_TYPE, printObject = true)
public class StringAppender extends AbstractOutputStreamAppender<StringAppender.StringOutputStreamManager> {
    private static LoggerContext context = (LoggerContext) LogManager.getContext(false);
    private static Configuration configuration = context.getConfiguration();
    private StringOutputStreamManager manager;

    private StringAppender(String name, Layout<? extends Serializable> layout, StringOutputStreamManager manager, boolean ignoreExceptions, boolean immediateFlush) {
        super(name, layout, null, ignoreExceptions, immediateFlush, null, manager);
        this.manager = manager;

        System.out.println ("StringAppender created");
    }

    public void addToLogger(final String loggerName, final Level level) {
        LoggerConfig loggerConfig = configuration.getLoggerConfig(loggerName);
        loggerConfig.addAppender(this, level, null);
        context.updateLoggers();
    }

    public void removeFromLogger(final String loggerName) {
        LoggerConfig loggerConfig = configuration.getLoggerConfig(loggerName);
        loggerConfig.removeAppender("StringAppender");
        context.updateLoggers();
    }

    public String getOutput() {
        manager.flush();
        return new String(manager.getStream().toByteArray());
    }

    @Override
    public boolean isFiltered(LogEvent event) {
        boolean rv = super.isFiltered(event);
        return rv;
    }

    @Override
    public void append(LogEvent event) {
        super.append(event);
    }

    @Override
    public void stop() {
        super.stop();
    }

    /**
     * Create a StringAppender with a given output format
     * @param nullablePatternString Can be {@code null}. The PatternLayout string for log output.
     * @return a new StringAppender
     */
    @PluginFactory
    public static StringAppender createStringAppender(@PluginAttribute("name") String name, @PluginElement("Filters") String nullablePatternString) {
        final ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        final PatternLayout layout;

        if (nullablePatternString == null) {
            layout = PatternLayout.createDefaultLayout();
        } else {
            layout = PatternLayout.newBuilder()
                    .withPattern(nullablePatternString)
                    .build();
        }

        return new StringAppender(
                name,
                layout,
                new StringOutputStreamManager(outputStream, "StringStream", layout),
                false,
                true);
    }

    /**
     * StringOutputStreamManager to manage an in memory byte-stream representing our stream
     */
    static class StringOutputStreamManager extends OutputStreamManager {
        ByteArrayOutputStream stream;
        StringOutputStreamManager(ByteArrayOutputStream os, String streamName, Layout<?> layout) {
            super(os, streamName, layout, false);
            stream = os;
        }

        @Override
        protected synchronized void writeToDestination(byte[] bytes, int offset, int length) {
            super.writeToDestination(bytes, offset, length);
        }

        @Override
        protected synchronized boolean closeOutputStream() {
            return super.closeOutputStream();
        }

        @Override
        protected void write(byte[] bytes, boolean immediateFlush) {
            super.write(bytes, immediateFlush);
        }

        @Override
        public void close() {
            super.close();
        }

        ByteArrayOutputStream getStream() {
            return stream;
        }
    }
}
