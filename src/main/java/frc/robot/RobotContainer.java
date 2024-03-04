package frc.robot;

import frc.robot.commands.swervedrive.drivebase.SuperSwerveDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDriveWithAimCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.ChameleonController;
import org.usfirst.frc3620.misc.ChameleonController.ControllerType;
import org.usfirst.frc3620.misc.JoystickAnalogButton;
import org.usfirst.frc3620.misc.DPad;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.XBoxConstants;
import org.usfirst.frc3620.misc.FlySkyConstants;

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
  private static VisionSubsystem visionSubsystem;

  public static List<Subsystem> allSubsystems = new ArrayList<>();

  private SuperSwerveController superSwerveController;

  // hardware here...
  private static DigitalInput practiceBotJumper;

  // joysticks here....
  public static Joystick operatorJoystick;
  public static ChameleonController driverJoystick;

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

    if (canDeviceFinder.isPowerDistributionPresent()) {
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
        () -> -getDriveVerticalJoystick(),
        () -> -getDriveHorizontalJoystick(),
        () -> -getDriveSpinJoystick(),
        () -> true);
    /*
     * TestDriveCommand StandardYagslDrive = new TestDriveCommand(drivebase,
     * () -> -getDriveVerticalJoystick(),
     * () -> -getDriveHorizontalJoystick(),
     * () -> -getDriveSpinJoystick(),
     * () -> true);
     * 
     * Command sitThereCommand = new TestDriveCommand(
     * drivebase,
     * () -> 0.0,
     * () -> 0.0,
     * () -> 0.0,
     * () -> false);
     */


    TeleopDriveWithAimCommand aimDrive = new TeleopDriveWithAimCommand(drivebase,
        () -> -getDriveVerticalJoystick(),
        () -> -getDriveHorizontalJoystick(),
        () -> -getDriveSpinJoystick(),
        () -> true,
        visionSubsystem,
        superSwerveController);

    drivebase.setDefaultCommand(aimDrive);

    if (drivebase.getCurrentCommand() != null) {
      SmartDashboard.putString("CurrentCommand", drivebase.getCurrentCommand().toString());
    } else {
      SmartDashboard.putString("CurrentCommand", "No Command");
    }
  }
    
  private void makeSubsystems() {
    visionSubsystem = new VisionSubsystem();
    addSubsystem(visionSubsystem);

    intakeSubsystem = new IntakeSubsystem();
    addSubsystem(intakeSubsystem);

    rollersSubsystem = new RollersSubsystem();
    addSubsystem(rollersSubsystem);

    climbElevationSubsystem = new ClimbElevationSubsystem();
    addSubsystem(climbElevationSubsystem);

    shooterSubsystem = new ShooterSubsystem();
    addSubsystem(shooterSubsystem);

    blinkySubsystem = new BlinkySubsystem(rollersSubsystem, visionSubsystem);
    addSubsystem(blinkySubsystem);
    //new InstantCommand(() -> blinkySubsystem.periodic());

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

  public String getDriverControllerName() {
    return driverJoystick.getName();
  }

  public void setDriverControllerName(ControllerType driveControllerType) {
    driverJoystick.setCurrentControllerType(driveControllerType);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    driverJoystick = new ChameleonController(new Joystick(0));
    operatorJoystick = new Joystick(1);

    DPad operatorDpad = new DPad(operatorJoystick, 0);

    // Driver controls
    if (drivebase != null) {
      // reset NavX
      driverJoystick.button(XBoxConstants.BUTTON_A, FlySkyConstants.BUTTON_SWA).onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

      // turn off "autoaiming" (robot does not try to keep shooter pointed @ speaker)
      driverJoystick.button(XBoxConstants.BUTTON_LEFT_BUMPER, FlySkyConstants.BUTTON_SWF).whileTrue(new InstantCommand(() -> drivebase.setAreWeAiming(false)));

      // X Mode
      driverJoystick.button(XBoxConstants.BUTTON_X, FlySkyConstants.BUTTON_SWD).whileTrue(new XModeCommand());
    }

    // intake
    driverJoystick.analogButton(XBoxConstants.AXIS_LEFT_TRIGGER, FlySkyConstants.AXIS_SWE).whileTrue(new RunRollersCommand(0.2));

    // well, shoot
    driverJoystick.analogButton(XBoxConstants.AXIS_RIGHT_TRIGGER, FlySkyConstants.AXIS_SWH).onTrue(new RunRollersUntilGone(0.8));

    // barf out a piece
    driverJoystick.analogButton(XBoxConstants.BUTTON_B, FlySkyConstants.BUTTON_SWC).whileTrue(new RunRollersCommand(-0.8));

    // Operator intake controls
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_LEFT_BUMPER)
        .onTrue(new GroundPickupCommand());

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A)
        .onTrue(new GroundToHomeCommand());

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_Y)
        .onTrue(new SetIntakeLocationCommand(IntakeLocation.ampPosition));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_B)
        .onTrue(new TrapShootCommand()
                .andThen(new WaitUntilCommand(() -> intakeSubsystem.getActualShoulderElevation() > 50))
                .andThen(new ActivateClimberJoystickCommand()));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_TRIGGER, 0.1)
        .toggleOnTrue(new SetShooterSpeedCommand(5000))
        .toggleOnTrue(new ShooterVisionAngleAdjustmentCommand(visionSubsystem, shooterSubsystem));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_START)
        .onTrue(new SetIntakeLocationCommand(IntakeLocation.homePosition))
        .onTrue(new SetShooterSpeedAndAngleCommand(ShooterSpeedAndAngle.shooterHome));
   
    operatorDpad.up().whileTrue(new ShoulderElevatePowerCommand(intakeSubsystem, 4));
    operatorDpad.down().whileTrue(new ShoulderElevatePowerCommand(intakeSubsystem, -4));
    operatorDpad.left().whileTrue(new ExtendPowerCommand(intakeSubsystem, -1.5));
    operatorDpad.right().whileTrue(new ExtendPowerCommand(intakeSubsystem, 1.5));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_Y)
        .onTrue(new SetClimberPowerPositionCommand());
  }

  private void setupSmartDashboardCommands() {
    SmartDashboard.putData("set shooter #1",
        new SetShooterSpeedAndAngleAndWaitCommand(ShooterSpeedAndAngle.testshooter1));
    SmartDashboard.putData("set shooter #2",
        new SetShooterSpeedAndAngleAndWaitCommand(ShooterSpeedAndAngle.testshooter2));
    SmartDashboard.putData("set variable shooter speed", new SetVariableShooterSpeedCommand());
    SmartDashboard.putData("set shooter wheels power", new ShooterWheelPowerCommand());
    SmartDashboard.putData("goInSixInches", new goInSixInchesCommand());

    if ( intakeSubsystem.getRequestedShoulderPosition() != null && intakeSubsystem.getRequestedExtendPosition() != null) {
      SmartDashboard.putData("Intake.elevate +", new SetIntakeLocationCommand(new IntakeLocation(intakeSubsystem.getRequestedShoulderPosition() + 2, intakeSubsystem.getActualExtendPosition(), intakeSubsystem.getActualWristPosition())));
      SmartDashboard.putData("Intake.elevate -", new SetIntakeLocationCommand(new IntakeLocation(intakeSubsystem.getRequestedShoulderPosition() - 2, intakeSubsystem.getActualExtendPosition(), intakeSubsystem.getActualWristPosition())));
      SmartDashboard.putData("Intake.extend +", new SetIntakeLocationCommand(new IntakeLocation(intakeSubsystem.getActualShoulderElevation(), intakeSubsystem.getRequestedExtendPosition() + 0.5, intakeSubsystem.getActualWristPosition())));
      SmartDashboard.putData("Intake.extend -", new SetIntakeLocationCommand(new IntakeLocation(intakeSubsystem.getActualShoulderElevation(), intakeSubsystem.getRequestedExtendPosition() - 0.5, intakeSubsystem.getActualWristPosition())));
    }

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
    SmartDashboard.putData("HomeToGroundPosition", new GroundPickupCommand());
    SmartDashboard.putData("GroundToHomePosition", new GroundToHomeCommand());
    SmartDashboard.putData("AmpShootCommand", new AmpShootCommand());

    SmartDashboard.putData("Climber to 0", new SetClimberPositionCommand(0));
    SmartDashboard.putData("Climber to 2", new SetClimberPositionCommand(2));

    SmartDashboard.putData("LockCamToTarget", new CameraLockToTargetTag(drivebase, visionSubsystem, superSwerveController));

    SmartDashboard.putData("Manually position intake", new PositionIntakeManuallyCommand());

    SmartDashboard.putData("Intake limp", new SetIntakeLocationCommand(new IntakeLocation(null, null, null)));

    SmartDashboard.putData("Intake elevate to 0", new SetIntakeLocationCommand(new IntakeLocation(0, 0, 0)));
    SmartDashboard.putData("Intake elevate to 10", new SetIntakeLocationCommand(new IntakeLocation(10, 0, 0)));
    SmartDashboard.putData("Intake elevate to 20", new SetIntakeLocationCommand(new IntakeLocation(20, 0, 0)));
    SmartDashboard.putData("Intake elevate to 40", new SetIntakeLocationCommand(new IntakeLocation(40, 0, 0)));

    SmartDashboard.putData("Intake elevate-extend-wrist to 10-0-0",
        new SetIntakeLocationCommand(new IntakeLocation(10, 0, 0)));
    SmartDashboard.putData("Intake elevate-extend-wrist to 10-0-4",
        new SetIntakeLocationCommand(new IntakeLocation(10, 0, 4)));
    SmartDashboard.putData("Intake elevate-extend to 10-2", new SetIntakeLocationCommand(new IntakeLocation(10, 2, 0)));
    SmartDashboard.putData("Intake elevate-extend to 10-6", new SetIntakeLocationCommand(new IntakeLocation(10, 6, 0)));
    SmartDashboard.putData("Intake elevate-extend to 10-8", new SetIntakeLocationCommand(new IntakeLocation(10, 8, 0)));
    SmartDashboard.putData("Intake elevate-extend to 10-14",
        new SetIntakeLocationCommand(new IntakeLocation(10, 14, 0)));
    SmartDashboard.putData("Intake elevate-extend to 75-0", new SetIntakeLocationCommand(new IntakeLocation(75, 0, 0)));

    // test rollers
    SmartDashboard.putData("Run Rollers until slurped", new RunRollersUntilDetected(0.8));
    SmartDashboard.putData("Run Rollers until gone", new RunRollersUntilGone(-0.8));

    // test Shooter
    SmartDashboard.putData("Test Shooter angle to 60",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 60)));
    SmartDashboard.putData("Test Shooter angle to 35",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 35)));
    SmartDashboard.putData("Test Shooter angle to 30",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 30)));
    SmartDashboard.putData("Test Shooter angle to 27.5",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 27.5)));
    SmartDashboard.putData("Test Shooter angle to 25",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 25)));

    SmartDashboard.putData("Test Shooter Angle PID zapper", new ShooterAnglePIDZapper());
    SmartDashboard.putData("Test Shooter Top PID zapper",
        new ShooterSpeedPIDZapper(shooterSubsystem.topMotor, "test.shooter.top"));
    SmartDashboard.putData("Test Shooter Bottom PID zapper",
        new ShooterSpeedPIDZapper(shooterSubsystem.bottomMotor, "test.shooter.bottom"));

    SmartDashboard.putData("Test Shooter Speed to 0",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(0, 60)));
    SmartDashboard.putData("Test Shooter Speed to 500",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(500, 60)));
    SmartDashboard.putData("Test Shooter Speed to 3000",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(3000, 60)));
    SmartDashboard.putData("Test Shooter Speed to 5000",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(5000, 60)));
    SmartDashboard.putData("Test Shooter Speed to 4500",
        new SetShooterSpeedAndAngleCommand(new ShooterSpeedAndAngle(4500, 60)));

    SmartDashboard.putData("Test Intake Angle PID Zapper", new IntakeAnglePIDZapper());
    SmartDashboard.putData("Test Intake Extension PID Zapper", new IntakeExtensionPIDZapper());

    SmartDashboard.putData("ShooterVisionAngleCommand", new ShooterVisionAngleAdjustmentCommand(visionSubsystem, shooterSubsystem));

  }

  
  SendableChooser<Command> chooser = new SendableChooser<>();

  public void setupAutonomousCommands() {
    chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto mode", chooser);

    chooser.addOption("Noop Command", new PrintCommand("no autonomous"));
    //chooser.addOption("Auto Command", drivebase.getAutonomousCommand("New Path", true));
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

  public static double getClimberJoystickPosition() {
    return operatorJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y);
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
    return axisValue;
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
    return axisValue;
  }
}