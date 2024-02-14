// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.junit.Test;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

public class LoggingTest {
    @Test
    public void test() {
        Logger logger = EventLogging.getLogger(LoggingTest.class, Level.INFO);
        System.out.println ("debug enabled = " + logger.isDebugEnabled());
        logger.debug("debug");
        logger.info("info");
        logger.warn("warn");
        logger.error("error");
    }
}
