[//]: # (render this with pandoc -V geometry:margin=0.5in -o Mason.pdf Mason.md)

# Getting ready for Mason

## Day to day

> **Note:**
> Make sure you pay attention to the list of things to do at the start and end of each work session.

### Start of session
* Make sure you have done a fetch.
* Make sure you are on the correct branch.
* Make sure you have pulled the correct branch.
* Make sure you have the correct identity pulled up with git-user-gui.

### End of session
* Make sure your files are saved in VSCode.
* Make sure you have committed (*including files you have added*).
* Make sure you push.
* Shutdown the PC, put it on it's charger if a laptop.

***

## Code tasks

### Path Planner

Continue to do all work on branch PathPlannerRepeatabilityTests.

- [ ] Stop using DoISeeSpeakerTag; do a canDistToSpeakerTag() and check for null.
- [ ] Refine autonomous heading.
- [ ] Develop more autonomous.

### Intake

Do all work on branch MasonIntake, start that branch from branch MasonStart.

- [ ] Simplify IntakeLocation.
- [ ] Remove all code related to the wrist (mechanism, commands)
- [ ] Remove all code related to the extension (mechanism, commands)
- [ ] The intake will probably just have two positions; in and out.
- [ ] Get rid of extra commands.
- [ ] Update controller diagrams and README.md.

### Vision

Do all work on branch MasonVision, start that branch from branch MasonStart.

- [ ] Remove PhotonVision dependency.
- [ ] Add the code for Limelight.
- [ ] Load field map into Limelight.
- [ ] Set 3D parameters for Limelight.
- [ ] Determine if Limelight tells us where we are.
- [ ] Get rid of DoISeeSpeakerTag.
- [ ] Do all the speaker calculations in one method, and save the data. Call that method from periodic().
All the other commands get the saved data.

### Shooter

Do all work on branch MasonShooter, start that branch from branch MasonStart.

- [ ] Add a motor for the bar. It will be a CANSparkMax. You will need to do position control. I recommend making it a mechanism the way we did the intake.
- [ ] Fix any commands that put pieces in the amplifier to extend the bar, run the shooter, and retract the bar.

### Swerve diagnostics

Do all work on branch MasonSwerveDiagnostics, start that brnach from branch MasonStart.

- [ ] Add commands to test (separately) the drive and angle motors and make sure they match; look at 2023. Those commands will need to have the SwerveSubsystem as a requirement. They need to put motor velocity and current draw up in SmartDashboard.
- [ ] Add a Dashboard pane that has buttons to run the test commands, and that will show the results of the tests.
- [ ] Some code will need to be added to SwerveSubsystem to run the motors. Doug can/will help.

### Telemetry

Do all work on branch MasonTelemetry, start that brnach from branch MasonStart.

- [ ] In RobotDataLogger, add code to log heading, odometry coordinates.

### Climber

Do all work on branch MasonClimber, start that brnach from branch MasonStart.

- [ ] Fix the scaling factor for the climber to match new mechanism (single stage vs double stage, possibly differe nt gearboxes).

### Autonomous 

Continue to do all work on branch MasonAuto.

- [ ] Figure out what to do if we miss a piece.
- [ ] other optimizations?

### Clear out old paths

Do all work on branch MasonCleanDeploy, start that brnach from branch MasonStart.

- [ ] Find a way to clear out the deploy directory on the roboRIO. Using Gradle looks promising. https://gradle-ssh-plugin.github.io/docs/
- [ ] **Possibly** find a way to make sure the cleaning happens on every deployments.

***

## March 14 Update

### Swerve

- [ ] Look at ramping azimuth motors to minimize belt skipping. Double check that it does not hurt PathPlanner accuracy.

### Vision

- [ ] Only do calculations on new data.

### Indexer subsystem

Do all work on branch MasonIndexer, start that brnach from branch MasonStart.

- [X] Write an indexer subsystem; it just needs to set up the indexer motor, and have a method on it to set the power.

### Intake

- [X] Need to write commands to bring the intake in and out. Commands will probably use PID to get the arm close to the end of travel, then just apply power to keep it there.

### Bar

- [X] Need to write commands to bring the bar in and out. Commands will probably use PID to get the bar close to the end of travel, then just apply power to keep it there.
- [ ] BarAnglePIDZapper.

### Dashboard

- [ ] Competition dashboard (vision, autonomous, a tab for diagnostics, ???)

***

## Before we give up the chassis

- [ ] Calibrate range calculation.
- [ ] Write out procedure for ensure that the camera is at the correct angle.

***

## When we get the chassis

### Bar

- [ ] Get bar going correct direction.
- [ ] Find out what the encoder readings are for the bar at the ends of it's travel, and fix the commands appropriately.
- [ ] Tune PID.

### Intake
- [ ] Get intake going correct direction.
- [ ] Find out what the intake readings are for the intake at the ends of it's travel, and fix the IntakeSubsystem.periodic() method appropriately.
- [ ] Tune PID (IntakeAnglePIDZapper)

### Climber
- [ ] Make sure climber is working, and initial deployment position is correct. Probably need to adjust source code.

### Indexer
- [ ] Make sure beam break works (use dashboard indicator "indexer.game_piece_detected").
- [ ] Get indexer going correct direction (use dashboard command "Test Indexer with joystick"). Make sure it stops when beam is broken.
- [ ] Make sure that we RunIndexerUntilGamePieceGoneCommand works (see dashboard commands "Test indexer until game piece detected" and "Test indexer until game piece gone").

### Swerve
- [ ] Diagnostics
- [ ] Horacio bar

***

## Driver Changes

* The operator right stick Y axis no longer "bumps" the intake position.
* The operator "retract intake" also shuts down the rollers and indexer.
