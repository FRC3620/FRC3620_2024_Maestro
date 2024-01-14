package org.usfirst.frc3620.logger;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LogCommand extends InstantCommand {
  static Logger defaultLogger = EventLogging.getLogger(LogCommand.class, Level.INFO);

  private Logger logger;

  private String msg;

  private Object[] args;

  public LogCommand(String message) {
    this(null, message, null);
  }

  public LogCommand(String message, Object[] args) {
    this(null, message, args);
  }

  public LogCommand(Logger constructorLogger, String message) {
    this(constructorLogger, message, null);
  }

  public LogCommand(Logger constructorLogger, String message, Object[] args) {
    logger = (constructorLogger == null) ? defaultLogger : constructorLogger;
    msg = message;
    this.args = args;
  }

  @Override
  public void initialize() {
    if (args == null) {
      logger.info(msg);
    } else {
      logger.info(msg, args);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
