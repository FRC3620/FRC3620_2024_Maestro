package org.usfirst.frc3620.misc;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

/**
 * Container for RobotParameters; designed to be subclassed.
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public class RobotParametersBase {
    protected String macAddress;
    protected boolean competitionRobot;
    protected String name;
    protected boolean makeAllCANDevices;

    public RobotParametersBase() {
        macAddress = "";
        competitionRobot = false;
        name = "";
        makeAllCANDevices = false;
    }

    public String getMacAddress() {
        return macAddress;
    }

    public boolean isCompetitionRobot() {
        return competitionRobot;
    }

    public String getName() {
        return name;
    }

    // TODO, looks like Jackson is not serializing this field.
    public boolean shouldMakeAllCANDevices() {
        return makeAllCANDevices;
    }

    @Override
    public String toString() {
        return super.toString() + " [macAddress=" + macAddress + ", competitionRobot=" + competitionRobot + ", name="
                + name + ", makeAllCANDevices=" + makeAllCANDevices + "]";
    }
}