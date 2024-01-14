package org.usfirst.frc3620.logger.log4j2;

import org.apache.logging.log4j.core.Core;
import org.apache.logging.log4j.core.Filter;
import org.apache.logging.log4j.core.LogEvent;

import org.apache.logging.log4j.core.config.plugins.Plugin;
import org.apache.logging.log4j.core.config.plugins.PluginAttribute;
import org.apache.logging.log4j.core.config.plugins.PluginFactory;
import org.apache.logging.log4j.core.filter.AbstractFilter;

@Plugin(name = "TimeIsSetFilter", category = Core.CATEGORY_NAME, elementType = Filter.ELEMENT_TYPE, printObject = true)
public final class TimeIsSetFilter extends AbstractFilter {
    private final static long SOME_TIME_AFTER_1970 = 523980000000L;

    int i = 0;
    private TimeIsSetFilter(Result onMatch, Result onMismatch) {
        super(onMatch, onMismatch);
    }

    @Override
    public Result filter(LogEvent event) {
        // Message m = event.getMessage();
        long ms = event.getTimeMillis();
        boolean timeIsSet = ms >= SOME_TIME_AFTER_1970;
        Result r = timeIsSet ? onMatch : onMismatch;
        // System.out.println ("ms=" + ms + ", timeIsSet=" + timeIsSet + ", m=" + m.getFormattedMessage() + ", r=" + r);
        return r;
    }

    /**
     * Create a ThresholdFilter.
     * @param onMatch The action to take on a match.
     * @param onMismatch The action to take on a mismatch.
     * @return The created ThresholdFilter.
     */
    @PluginFactory
    public static TimeIsSetFilter createFilter(@PluginAttribute(value = "onMatch", defaultString = "NEUTRAL") Result onMatch,
                                               @PluginAttribute(value = "onMismatch", defaultString= "DENY") Result onMismatch) {
        return new TimeIsSetFilter(onMatch, onMismatch);
    }}