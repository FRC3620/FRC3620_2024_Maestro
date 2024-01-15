package org.usfirst.frc3620.misc;

import java.util.StringJoiner;

import com.fasterxml.jackson.annotation.JsonProperty;

public class SwerveParameters  {
    @JsonProperty("he.rf")
    private Double rightFrontAbsoluteOffset;

    @JsonProperty("he.lf")
    private Double leftFrontAbsoluteOffset;

    @JsonProperty("he.lb")
    private Double leftBackAbsoluteOffset;

    @JsonProperty("he.rb")
    private Double rightBackAbsoluteOffset;

    @JsonProperty("chassis_width")
    private Double chassisWidth;

    @JsonProperty("chassis_length")
    private Double chassisLength;

    @JsonProperty("drive_gear_ratio")
    private Double driveGearRatio;

    public SwerveParameters() {
        super();
    }

    public Double getRightFrontAbsoluteOffset() {
        return rightFrontAbsoluteOffset;
    }

    public Double getLeftFrontAbsoluteOffset() {
        return leftFrontAbsoluteOffset;
    }

    public Double getRightBackAbsoluteOffset() {
        return rightBackAbsoluteOffset;
    }

    public Double getLeftBackAbsoluteOffset() {
        return leftBackAbsoluteOffset;
    }

    public Double getChassisLength() {
        return chassisLength;
    }

    public Double getChassisWidth() {
        return chassisWidth;
    }

    public Double getDriveGearRatio() {
        return driveGearRatio;
    }

    public String whichSwerveParametersAreMissing() {
        StringJoiner l = new StringJoiner("|");
        if (leftFrontAbsoluteOffset == null) l.add("swerve.he.lf");
        if (rightFrontAbsoluteOffset == null) l.add("swerve.he.rf");
        if (leftBackAbsoluteOffset == null) l.add("swerve.he.lb");
        if (rightBackAbsoluteOffset == null) l.add("swerve.he.rb");
        if (chassisLength == null) l.add("swerve.chassis_length");
        if (chassisWidth == null) l.add("swerve.chassis_width");
        if (driveGearRatio == null) l.add("swerve.drive_gear_ratio");
        if (l.length() == 0) return null;
        return l.toString();
    }

    @Override
    public String toString() {
        return new StringJoiner(", ", SwerveParameters.class.getSimpleName() + "[", "]")
                .add("rightFrontAbsoluteOffset=" + rightFrontAbsoluteOffset)
                .add("leftFrontAbsoluteOffset=" + leftFrontAbsoluteOffset)
                .add("leftBackAbsoluteOffset=" + leftBackAbsoluteOffset)
                .add("rightBackAbsoluteOffset=" + rightBackAbsoluteOffset)
                .add("chassisWidth=" + chassisWidth)
                .add("chassisLength=" + chassisLength)
                .add("driveGearRatio=" + driveGearRatio)
                .toString();
    }
}