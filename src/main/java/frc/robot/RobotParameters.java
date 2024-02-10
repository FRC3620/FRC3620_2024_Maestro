package frc.robot;

import org.usfirst.frc3620.misc.RobotParametersBase;

/**
 * add members here as needed
 */
public class RobotParameters extends RobotParametersBase {
    String swerveDirectoryName;

    public String getSwerveDirectoryName() {
        return swerveDirectoryName;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(super.toString());
        sb.setLength(sb.length()-1);
        sb.append (", swerveDirectoryName=" + swerveDirectoryName);
        sb.append ("]");
        return sb.toString();
    }

    
}