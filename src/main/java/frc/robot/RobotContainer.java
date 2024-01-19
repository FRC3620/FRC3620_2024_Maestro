package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.INavigationSubsystem;
import frc.robot.subsystems.NavXNavigationSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.ChameleonController;
import org.usfirst.frc3620.misc.ChameleonController.ControllerType;
import org.usfirst.frc3620.misc.FlySkyConstants;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.XBoxConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final static Logger logger = EventLogging.getLogger(RobotContainer.class, Level.INFO);

  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  // hardware here...
  private static DigitalInput practiceBotJumper;

  public static PneumaticsModuleType pneumaticModuleType = null;

  // subsystems here
  public static INavigationSubsystem navigationSubsystem;
  @SuppressWarnings("unused") private static DriveSubsystem driveSubsystem;

  // joysticks here....
  public static Joystick rawDriverJoystick;
  public static ChameleonController driverJoystick;
  public static Joystick operatorJoystick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();

    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("got parameters for chassis '{}'", robotParameters.getName());

    practiceBotJumper = new DigitalInput(9);
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || iAmACompetitionRobot) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }

    makeSubsystems();

    // Configure the button bindings
    configureButtonBindings();

    setupSmartDashboardCommands();

    setupAutonomousCommands();
  }

  private void makeSubsystems() {
    navigationSubsystem = new NavXNavigationSubsystem();
    driveSubsystem = new DriveSubsystem(navigationSubsystem);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    rawDriverJoystick = new Joystick(0);
    driverJoystick = new ChameleonController(rawDriverJoystick);
    operatorJoystick = new Joystick(1);

  }

  private void setupSmartDashboardCommands() {
    // SmartDashboard.putData(new xxxxCommand());
  }

  SendableChooser<Command> chooser = new SendableChooser<>();

  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);

    chooser.addOption("Noop Command", new PrintCommand("no autonomous"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new GoldenAutoCommand(driveSubsystem, shooterSubsystem,
    // VisionSubsystem, intakeSubsystem);
    return chooser.getSelected();
  }

  /**
   * Determine if this robot is a competition robot.
   *
   * It is if it's connected to an FMS.
   *
   * It is if it is missing a grounding jumper on DigitalInput 0.
   *
   * It is if the robot_parameters.json says so for this MAC address.
   *
   * @return true if this robot is a competition robot.
   */
  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if (practiceBotJumper.get() == true) {
      return true;
    }

    if (robotParameters.isCompetitionRobot()) {
      return true;
    }

    return false;
  }

  /**
   * Determine if we should make software objects, even if the device does
   * not appear on the CAN bus.
   *
   * We should if it's connected to an FMS.
   *
   * We should if it is missing a grounding jumper on DigitalInput 0.
   *
   * We should if the robot_parameters.json says so for this MAC address.
   * 
   * @return true if we should make all software objects for CAN devices
   */
  public static boolean shouldMakeAllCANDevices() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if (practiceBotJumper.get() == true) {
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

  public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y, FlySkyConstants.AXIS_LEFT_Y);
    double deadzone = 0.1;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadzone = 0.05;
    }
    SmartDashboard.putNumber("driver.y.raw", axisValue);
    if (Math.abs(axisValue) < deadzone) {
      return 0;
    }
    if (axisValue < 0) {
      return (axisValue * axisValue);
    }
    return -axisValue * axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X, FlySkyConstants.AXIS_LEFT_X);
    double deadzone = 0.1;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadzone = 0.05;
    }
    SmartDashboard.putNumber("driver.x.raw", axisValue);
    if (Math.abs(axisValue) < deadzone) {
      return 0;
    }
    if (axisValue < 0) {
      return -(axisValue * axisValue);
    }
    return axisValue * axisValue;
  }

  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X, FlySkyConstants.AXIS_RIGHT_X);
    double deadzone = 0.1;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadzone = 0.05;
    }
    SmartDashboard.putNumber("driver.spin.raw", axisValue);
    double rv = 0;
    if (Math.abs(axisValue) >= deadzone) {
      rv = axisValue * axisValue;
      if (axisValue < 0) {
        rv = -rv;
      }
    }

    /*
     * this should not be necessary, but autonomous code is looking at
     * the spin joystick, so here we are...
     */
    if (Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
      rv = 0;
    }

    SmartDashboard.putNumber("driver.spin.processed", rv);
    return rv;
  }

}