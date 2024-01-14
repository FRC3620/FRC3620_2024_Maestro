package org.usfirst.frc3620.logger.log4j2;

import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.core.*;
import org.apache.logging.log4j.core.appender.AbstractAppender;
import org.apache.logging.log4j.core.config.plugins.Plugin;
import org.apache.logging.log4j.core.config.plugins.PluginAttribute;
import org.apache.logging.log4j.core.config.plugins.PluginElement;
import org.apache.logging.log4j.core.config.plugins.PluginFactory;
import org.apache.logging.log4j.core.layout.PatternLayout;

import edu.wpi.first.hal.DriverStationJNI;

import java.io.Serializable;

@Plugin(name = "DriverStation", category = Core.CATEGORY_NAME, elementType = Appender.ELEMENT_TYPE, printObject = true)
public class DriverStationAppender extends AbstractAppender {
    StackTraceElement[] noStackTrace = new StackTraceElement[0];

    public DriverStationAppender(final String name, final Filter filter, final Layout<? extends Serializable> layout) {
        super(name, filter, layout, true, null);
    }

    @PluginFactory
    public static DriverStationAppender createAppender(@PluginAttribute("name") String name,
                                                       @PluginElement("Layout") Layout<? extends Serializable> layout,
                                                       @PluginElement("Filters") Filter filter) {
        if (layout == null) {
            layout = PatternLayout.createDefaultLayout();
        }

        return new DriverStationAppender(name, filter, layout);
    }

    @Override
    public void append(final LogEvent event) {
        byte[] ba = getLayout().toByteArray(event);
        String s = new String(ba);
        // System.out.print ("DS1 on thread " + Thread.currentThread().getName());
        Level level = event.getLevel();
        String ls = level.toString();
        if (level.equals(Level.WARN)) {
            ls = "WARNING";
        }
        DriverStationJNI.sendConsoleLine(ls + " " + s);
    }
}