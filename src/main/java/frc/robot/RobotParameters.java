package frc.robot;

import org.usfirst.frc3620.misc.RobotParametersBase;
import org.usfirst.frc3620.misc.SwerveParameters;

/**
 * add members here as needed
 */
public class RobotParameters extends RobotParametersBase {
    SwerveParameters swerveParameters;

    public SwerveParameters getSwerveParameters() {
        return swerveParameters;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(super.toString());
        sb.setLength(sb.length()-1);
        sb.append (", swerveParameters=" + swerveParameters);
        sb.append ("]");
        return sb.toString();
    }

}