package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.SuperSwerveDrive;
import frc.robot.commands.swervedrive.drivebase.TestDriveCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.io.File;
import java.util.Set;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.JoystickAnalogButton;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.XBoxConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  public final static Logger logger = EventLogging.getLogger(RobotContainer.class, Level.INFO);
  
  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  // subsystems here
  public static IntakeSubsystem intakeSubsystem;
  public static ClimbElevationSubsystem climbElevationSubsystem;

  private SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve"));

  private final SuperSwerveController superSwerveController = new SuperSwerveController(drivebase);

  // hardware here...
  private static DigitalInput practiceBotJumper;

  public static PneumaticsModuleType pneumaticModuleType = null;

  private static DriveSubsystem driveSubsystem;
  public ShooterSubsystem shooterSubsystem;

  // joysticks here....
  CommandJoystick driverController = new CommandJoystick(1);
  XboxController driverXbox = new XboxController(0);
public static BlinkySubsystem blinkySubsystem;
  public static Joystick operatorJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();
    blinkySubsystem=new BlinkySubsystem();
    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info ("got parameters for chassis '{}'", robotParameters.getName());

    practiceBotJumper = new DigitalInput(0);
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    /*
    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || iAmACompetitionRobot) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }
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
                                                    () -> -driverXbox.getRawAxis(4), () -> true);

    Command sitThereCommand = new TestDriveCommand(
        drivebase,
        () -> 0.0,
        () -> 0.0,
        () -> 0.0,
        () -> false
    );

    drivebase.setDefaultCommand(SuperFieldRel);


  }

  private void makeSubsystems() {
    intakeSubsystem = new IntakeSubsystem();
    climbElevationSubsystem= new ClimbElevationSubsystem();
    shooterSubsystem=new ShooterSubsystem();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // Operator intake controls
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_LEFT_BUMPER)
            .onTrue(new SetIntakeLocationCommand(IntakeLocation.groundPosition));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A)
            .onTrue(new SetIntakeLocationCommand(IntakeLocation.homePosition));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_Y)
            .onTrue(new SetIntakeLocationCommand(IntakeLocation.ampPosition));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_TRIGGER)
            .onTrue(new SetIntakeLocationCommand(IntakeLocation.trapPosition));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_TRIGGER)
            .onTrue(new SetIntakeLocationCommand(IntakeLocation.preclimbPosition));
  }

  private void setupSmartDashboardCommands() {
    // SmartDashboard.putData(new xxxxCommand());
    SmartDashboard.putData("set shooter speed", new SetShooterSpeedCommand(10, shooterSubsystem));
    SmartDashboard.putData("set shooter speed+wait", new SetShooterSpeedAndWaitCommand(shooterSubsystem, 15));
    SmartDashboard.putData("set variable shooter speed", new SetVariableShooterSpeedCommand(shooterSubsystem));

    SmartDashboard.putData("GroundPosition", new SetIntakeLocationCommand(IntakeLocation.groundPosition));
    SmartDashboard.putData("HomePosition", new SetIntakeLocationCommand(IntakeLocation.homePosition));
    SmartDashboard.putData("AmpPosition", new SetIntakeLocationCommand(IntakeLocation.ampPosition));
    SmartDashboard.putData("TrapPosition", new SetIntakeLocationCommand(IntakeLocation.trapPosition));
    SmartDashboard.putData("PreclimbPosition", new SetIntakeLocationCommand(IntakeLocation.preclimbPosition));
    
    SmartDashboard.putData("Climber to 0", new SetClimberPositionCommand(0));
    SmartDashboard.putData("Climber to 2", new SetClimberPositionCommand(2));

  }

  SendableChooser<Command> chooser = new SendableChooser<>();
  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);

    chooser.addOption("Noop Command", new PrintCommand("no autonomous"));
    //chooser.addOption("Auto Command", drivebase.getAutonomousCommand("New Path", true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new GoldenAutoCommand(driveSubsystem, shooterSubsystem, VisionSubsystem, intakeSubsystem);
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

    if(practiceBotJumper.get() == true){
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

    if(practiceBotJumper.get() == true){
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}