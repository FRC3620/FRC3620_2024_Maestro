package frc.robot.blinky;

import frc.robot.subsystems.BlinkySubsystem.LightSegment;

/** Add your docs here. */
abstract public class Pattern{
    public void start (LightSegment lightSegment) { }

    public boolean periodic (LightSegment lightSegment) { return false; }

    public void done (LightSegment lightSegment) { }
}
