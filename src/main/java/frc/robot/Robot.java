package frc.robot;

import java.util.function.Consumer;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.FileSaver;
import org.usfirst.frc3620.misc.GitNess;
import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Logger logger;

  static private RobotMode currentRobotMode = RobotMode.INIT, previousRobotMode;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    logger = EventLogging.getLogger(Robot.class, Level.INFO);
    logger.info ("I'm alive! {}", GitNess.gitDescription());

    PortForwarder.add (10080, "wpilibpi.local", 80);
    PortForwarder.add (10022, "wpilibpi.local", 22);

    CommandScheduler.getInstance().onCommandInitialize(new Consumer<Command>() {//whenever a command initializes, the function declared bellow will run.
      public void accept(Command command) {
        logger.info("Initialized {}", command.getClass().getSimpleName());//I scream at people
      }
    });

    CommandScheduler.getInstance().onCommandFinish(new Consumer<Command>() {//whenever a command ends, the function declared bellow will run.
      public void accept(Command command) {
        logger.info("Ended {}", command.getClass().getSimpleName());//I, too, scream at people
      }
    });

    CommandScheduler.getInstance().onCommandInterrupt(new Consumer<Command>() {//whenever a command ends, the function declared bellow will run.
      public void accept(Command command) {
        logger.info("Interrupted {}", command.getClass().getSimpleName());//I, in addition, as well, scream.
      }
    });
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // get data logging going
    DataLogger robotDataLogger = new DataLogger();
    new RobotDataLogger(robotDataLogger, RobotContainer.canDeviceFinder);
    robotDataLogger.setInterval(0.25);
    robotDataLogger.start();

    FileSaver.add("networktables.ini");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    processRobotModeChange(RobotMode.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    logCANBusIfNecessary(); // don't do this when enabled; unnecessary overhead
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    logCANBusIfNecessary();

    processRobotModeChange(RobotMode.AUTONOMOUS);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    logCANBusIfNecessary();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    processRobotModeChange(RobotMode.TELEOP);
    logMatchInfo();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    logCANBusIfNecessary();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    processRobotModeChange(RobotMode.TEST);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /*
  * this routine gets called whenever we change modes
  */
  void processRobotModeChange(RobotMode newMode) {
    logger.info("Switching from {} to {}", currentRobotMode, newMode);
    
    previousRobotMode = currentRobotMode;
    currentRobotMode = newMode;

    // if any subsystems need to know about mode changes, let
    // them know here.
    // exampleSubsystem.processRobotModeChange(newMode);
    
  }

  public static RobotMode getCurrentRobotMode(){
    return currentRobotMode;
  }

  public static RobotMode getPreviousRobotMode(){
    return previousRobotMode;
  }

  void logMatchInfo() {
    if (DriverStation.isFMSAttached()) {
      logger.info("FMS attached. Event name {}, match type {}, match number {}, replay number {}", 
        DriverStation.getEventName(),
        DriverStation.getMatchType(),
        DriverStation.getMatchNumber(),
        DriverStation.getReplayNumber());
    }
    logger.info("Alliance {}, position {}", DriverStation.getAlliance(), DriverStation.getLocation());
  }

  private final static long SOME_TIME_AFTER_1970 = 523980000000L;
  private boolean hasCANBusBeenLogged;
  
  void logCANBusIfNecessary() {
    if (!hasCANBusBeenLogged) {
      long now = System.currentTimeMillis();
      if (now > SOME_TIME_AFTER_1970) {
        logger.info ("CAN bus: " + RobotContainer.canDeviceFinder.getDeviceSet());
        var missingDevices = RobotContainer.canDeviceFinder.getMissingDeviceSet();
        if (missingDevices.size() > 0) {
          logger.warn ("Missing devices: " + missingDevices);
        }
        hasCANBusBeenLogged = true;
      }
    }
  }

}