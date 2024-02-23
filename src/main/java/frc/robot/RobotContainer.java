package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.SuperSwerveDrive;
import frc.robot.commands.swervedrive.drivebase.TestDriveCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.*;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.JoystickAnalogButton;
import org.usfirst.frc3620.misc.DPad;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.XBoxConstants;

import com.pathplanner.lib.auto.AutoBuilder;

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

  // need these
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;
  public static PowerDistribution powerDistribution;

  // subsystems here
  public static IntakeSubsystem intakeSubsystem;
  public static RollersSubsystem rollersSubsystem;
  public static ClimbElevationSubsystem climbElevationSubsystem;
  public static ShooterSubsystem shooterSubsystem;
  public static BlinkySubsystem blinkySubsystem;
  public static SwerveSubsystem drivebase;
  public static SwerveMotorTestSubsystem swerveMotorTestSubsystem;

  public static List<Subsystem> allSubsystems = new ArrayList<>();

  private SuperSwerveController superSwerveController;

  // hardware here...
  private static DigitalInput practiceBotJumper;

  // joysticks here....
  public static XboxController driverXbox;
  public static Joystick operatorJoystick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Robot.printMemoryStatus("starting CANDeviceFinder");
    canDeviceFinder = new CANDeviceFinder();
    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("got parameters for chassis '{}'", robotParameters.getName());

    practiceBotJumper = new DigitalInput(0);
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    if (canDeviceFinder.isPowerDistributionPresent() || true) {
      logger.info("We have a PowerDistributionPanel");
      powerDistribution = new PowerDistribution();
      if (powerDistribution == null) {
        logger.error("...but we couldn't make the object!");
      } else {
        logger.error("...and got a WPILIB object for it: {} {}", powerDistribution.getClass(),
            powerDistribution.getType());
      }
    } else {
      logger.error("Missing power distribution panel");
    }

    /*
     * if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") ||
     * iAmACompetitionRobot) {
     * pneumaticModuleType = PneumaticsModuleType.REVPH;
     * } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0,
     * "CTRE PCM")) {
     * pneumaticModuleType = PneumaticsModuleType.CTREPCM;
     * }
     */

    makeSubsystems();

    canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 1, "RF Drive");

    // CAN bus ok?
    Set<CANDeviceFinder.NamedCANDevice> missingDevices = canDeviceFinder.getMissingDeviceSet();
    if (missingDevices.size() > 0) {
      SmartDashboard.putBoolean("can.ok", false);
      SmartDashboard.putString("can.missing", missingDevices.toString());
    } else {
      SmartDashboard.putBoolean("can.ok", true);
      SmartDashboard.putString("can.missing", "");
    }

    // Configure the button bindings
    configureButtonBindings();

    setupSmartDashboardCommands();

    setupAutonomousCommands();

    SuperSwerveDrive SuperFieldRel = new SuperSwerveDrive(drivebase,
        superSwerveController,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(4),
            OperatorConstants.RIGHT_X_DEADBAND),
        () -> true);

    TestDriveCommand StandardYagslDrive = new TestDriveCommand(drivebase,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(4), () -> true);

    Command sitThereCommand = new TestDriveCommand(
        drivebase,
        () -> 0.0,
        () -> 0.0,
        () -> 0.0,
        () -> false);

    drivebase.setDefaultCommand(SuperFieldRel);

    if (drivebase.getCurrentCommand() != null) {
      SmartDashboard.putString("CurrentCommand", drivebase.getCurrentCommand().toString());
    } else {
      SmartDashboard.putString("CurrentCommand", "No Command");
    }
  }

  private void makeSubsystems() {
    intakeSubsystem = new IntakeSubsystem();
    addSubsystem(intakeSubsystem);

    rollersSubsystem = new RollersSubsystem();
    addSubsystem(rollersSubsystem);

    climbElevationSubsystem = new ClimbElevationSubsystem();
    addSubsystem(climbElevationSubsystem);

    shooterSubsystem = new ShooterSubsystem();
    addSubsystem(shooterSubsystem);

    blinkySubsystem = new BlinkySubsystem();
    addSubsystem(blinkySubsystem);

    Robot.printMemoryStatus("making drivebase");

    String swerveFolder = robotParameters.getSwerveDirectoryName();
    if (swerveFolder == null)
      swerveFolder = "compbot";
    SmartDashboard.putString("swerveFolder", swerveFolder);
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveFolder));
    addSubsystem(drivebase);

    Robot.printMemoryStatus("making superSwerveController");

    superSwerveController = new SuperSwerveController(drivebase);

    Robot.printMemoryStatus("making subsystems");

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
    driverXbox = new XboxController(0);
    operatorJoystick = new Joystick(1);
    DPad driverDPad = new DPad(driverXbox, 0);

    if (drivebase != null) {
      // Driver controls
      new JoystickButton(driverXbox, XBoxConstants.BUTTON_A).onTrue(new InstantCommand(drivebase::zeroGyro));
      new JoystickButton(driverXbox, XBoxConstants.BUTTON_X)
          .onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    }

    // Operator intake controls
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_LEFT_BUMPER)
        .onTrue(new PositionIntakeManuallyCommand(IntakeLocation.groundPosition));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A)
        .onTrue(new PositionIntakeManuallyCommand(IntakeLocation.homePosition));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_Y)
        .onTrue(new PositionIntakeManuallyCommand(IntakeLocation.ampPosition));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.BUTTON_B)
        .onTrue(new PositionIntakeManuallyCommand(IntakeLocation.preclimbPosition));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_TRIGGER)
        .onTrue(new SetShooterSpeedAndAngleCommand(ShooterSpeedAndAngle.subWoofShot));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_RIGHT_BUMPER)
        .onTrue(new SetShooterSpeedAndAngleCommand(ShooterSpeedAndAngle.shootingPosition));

    driverDPad.up().onTrue(new TurnToCommand(drivebase, superSwerveController, 0));
    driverDPad.right().onTrue(new TurnToCommand(drivebase, superSwerveController, 90));
    driverDPad.down().onTrue(new TurnToCommand(drivebase, superSwerveController, 180));
    driverDPad.left().onTrue(new TurnToCommand(drivebase, superSwerveController, -90));

  }

  private void setupSmartDashboardCommands() {
    // SmartDashboard.putData("set shooter speed", new
    // SetShooterSpeedAndAngleCommand(10, shooterSubsystem));
    SmartDashboard.putData("set shooter #1",
        new SetShooterSpeedAndAngleAndWaitCommand(ShooterSpeedAndAngle.testshooter1));
    SmartDashboard.putData("set shooter #2",
        new SetShooterSpeedAndAngleAndWaitCommand(ShooterSpeedAndAngle.testshooter2));
    SmartDashboard.putData("set variable shooter speed", new SetVariableShooterSpeedCommand());
    SmartDashboard.putData("set shooter wheels power", new ShooterWheelPowerCommand());

    /*
     * SmartDashboard.putData("GroundPosition", new
     * SetIntakeLocationCommand(IntakeLocation.groundPosition));
     * SmartDashboard.putData("HomePosition", new
     * SetIntakeLocationCommand(IntakeLocation.homePosition));
     * SmartDashboard.putData("AmpPosition", new
     * SetIntakeLocationCommand(IntakeLocation.ampPosition));
     * SmartDashboard.putData("TrapPosition", new
     * SetIntakeLocationCommand(IntakeLocation.trapPosition));
     * SmartDashboard.putData("PreclimbPosition", new
     * SetIntakeLocationCommand(IntakeLocation.preclimbPosition));
     */

    SmartDashboard.putData("Climber to 0", new SetClimberPositionCommand(0));
    SmartDashboard.putData("Climber to 2", new SetClimberPositionCommand(2));

    SmartDashboard.putData("Intake elevate to 0", new PositionIntakeManuallyCommand(new IntakeLocation(0, 0, 0)));
    SmartDashboard.putData("Intake elevate to 10", new PositionIntakeManuallyCommand(new IntakeLocation(10, 0, 0)));
    SmartDashboard.putData("Intake elevate to 20", new PositionIntakeManuallyCommand(new IntakeLocation(20, 0, 0)));
    SmartDashboard.putData("Intake elevate to 40", new PositionIntakeManuallyCommand(new IntakeLocation(40, 0, 0)));

    SmartDashboard.putData("Intake elevate-extend-wrist to 10-0-0",
        new PositionIntakeManuallyCommand(new IntakeLocation(10, 0, 0)));
    SmartDashboard.putData("Intake elevate-extend-wrist to 10-0-4",
        new PositionIntakeManuallyCommand(new IntakeLocation(10, 0, 4)));
    SmartDashboard.putData("Intake elevate-extend to 10-2", new PositionIntakeManuallyCommand(new IntakeLocation(10, 2, 0)));
    SmartDashboard.putData("Intake elevate-extend to 10-6", new PositionIntakeManuallyCommand(new IntakeLocation(10, 6, 0)));
    SmartDashboard.putData("Intake elevate-extend to 10-8", new PositionIntakeManuallyCommand(new IntakeLocation(10, 8, 0)));
    SmartDashboard.putData("Intake elevate-extend to 10-14",
        new PositionIntakeManuallyCommand(new IntakeLocation(10, 14, 0)));
    SmartDashboard.putData("Intake elevate-extend to 75-0", new PositionIntakeManuallyCommand(new IntakeLocation(75, 0, 0)));

    // test rollers
    SmartDashboard.putData("Run Rollers until slurped", new RunRollersUntilDetected());
    SmartDashboard.putData("Run Rollers until gone", new RunRollersUntilGone());

    // test Shooter
    SmartDashboard.putData("Shooter speed-angle to 0-0",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 0)));
    SmartDashboard.putData("Shooter speed-angle to 0.3-30",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(.3, 30)));
  }

  SendableChooser<Command> chooser = new SendableChooser<>();

  public void setupAutonomousCommands() {
    chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto mode", chooser);

    // chooser.addOption("Noop Command", new PrintCommand("no autonomous"));
    // chooser.addOption("PathPlannerAuto", getAutonomousCommand());
    // chooser.addOption("Auto Command", drivebase.driveToPose(new Pose2d(new
    // Translation2d(1.5, 0), new Rotation2d(0))));
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

  void addSubsystem(Subsystem subsystem) {
    allSubsystems.add(subsystem);
  }

}