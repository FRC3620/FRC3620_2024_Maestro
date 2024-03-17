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

## JoeHann Return-to-Service Checklist

### 1. Teleop Drive
- [ ] Horatio Bar all wheel module
- [ ] (On the Cart) Validate that basic drive functions work (translational and rotational)
- [ ] (On the Cart) Eyeball azimuth motor PID values (does the module oscillate when rotating?)

### 2. Intake Subsystem
- [ ] Verify Intake "Down" and "Up" Encoder positions
- [ ] Verify Intake Initialization / zero-position
- [ ] Test Command to extend / retract intake
- [ ] Test Intake Roller Command

### 3. Indexer
- [ ] Need to Merge MasonIndexer into MasonStart
- [ ] Verify Indexer direction. Positive power should move note toward shooter.
- [ ] ? Is Indexer running with Intake? How are we shutting off Indexer before shooting?
- [ ] Create "Shoot" command to move the note from Indexer into Shooter

### 4. Shooter
- [ ] Validate that Shooter still spins up properly
- [ ] Validate that the Shooter Elevation moves properly

### 5. Climber
- [ ] Validate that Climber re-zeroes properly during init
- [ ] Check new "Up" and "Down" encoder positions & update command(s)

### 6. Vision
- [ ] Mount Camera to JoeHann
- [ ] Update Limelight settings (Camera height, angle, position, etc...)
- [ ] Place robot on field and validate that camDistToSpeakerTag returns the correct value
- [ ] Validate that camYawtoSpeakerTag returns the correct value

### 7. Putting it all together (Teleop)
- [ ] Validate that we can pick up a note and hold it in the indexer
- [ ] Validate that we can transfer a note from the indexer to the shooter
- [ ] Validate that we can successfully shoot from the base of the subwoofer
- [ ] Validate that the shooter elevation moves properly according to our distance from the speaker
- [ ] Validate that our aimDrive accurately points our shooter towards the speaker

### 8. Putting it all together (Auto)
- [ ] Run our "Straight" path and validate PID for auto
- [ ] Test our three note path and make sure the path itself is viable
- [ ] Update the PickUpAndShoot commands to pick up and shoot all in one step as long as we have a solution



## Code tasks

### Path Planner

Continue to do all work on branch PathPlannerRepeatabilityTests.

- [ ] Stop using DoISeeSpeakerTag; do a canDistToSpeakerTag() and check for null.
- [X] Refine autonomous heading.
- [ ] Develop more autonomous.

### Intake

Do all work on branch MasonIntake, start that branch from branch MasonStart.

- [ ] Simplify IntakeLocation.
- [X] Remove all code related to the wrist (mechanism, commands)
- [X] Remove all code related to the extension (mechanism, commands)
- [ ] The intake will probably just have two positions; in and out.
- [ ] Get rid of extra commands.
- [ ] Update controller diagrams and README.md.

### Vision

Do all work on branch MasonVision, start that branch from branch MasonStart.

- [X] Remove PhotonVision dependency.
- [X] Add the code for Limelight.
- [ ] Load field map into Limelight.
- [ ] Set 3D parameters for Limelight.
- [ ] Determine if Limelight tells us where we are.
- [X] Get rid of DoISeeSpeakerTag.
- [X] Do all the speaker calculations in one method, and save the data. Call that method from periodic().
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

- [ ] Write an indexer subsystem; it just needs to set up the indexer motor, and have a method on it to set the power.

### Intake

- [ ] Need to write commands to bring the intake in and out. Commands will probably use PID to get the arm close to the end of travel, then just apply power to keep it there.

### Bar

- [ ] Need to write commands to bring the bar in and out. Commands will probably use PID to get the bar close to the end of travel, then just apply power to keep it there.

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
- [ ] Find out what the intake readings are for the intake at the ends of it's travel, and fix the commands appropriately.
- [ ] Tune PID.

### Climber
- [ ] Make sure climber is working, and initial deployment position is correct.

### Indexer
- [ ] Get indexer going correct direction.
