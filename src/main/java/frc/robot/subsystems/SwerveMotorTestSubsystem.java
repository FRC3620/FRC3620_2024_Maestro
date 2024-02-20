package frc.robot.subsystems;

import java.util.*;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANSparkMaxSendableWrapper;
import org.usfirst.frc3620.misc.Utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;

/** Add your docs here. */
public class SwerveMotorTestSubsystem extends SubsystemBase {
    Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

    Map<String, Object> motors = new TreeMap<>();

    public SwerveMotorTestSubsystem (SwerveDrive swerveDrive, int sm) {
        // Utilities.dumpSendables("startup", getName());

        SwerveModule[] modules = swerveDrive.getModules();
        NS ns;
        for (var module : modules) {
            String moduleName = module.getConfiguration().name;
            ns = makeRealSendable(moduleName, module.getAngleMotor(), "angle");
            if (ns != null) {
                addChild (ns.n, ns.s);
                Utilities.dumpSendables("after " + ns.n, getName());
            }
            ns = makeRealSendable(moduleName, module.getDriveMotor(), "drive");
            if (ns != null) {
                addChild (ns.n, ns.s);
                Utilities.dumpSendables("after " + ns.n, getName());
            }
        }
        logger.info(motors.toString());
    }

    NS makeRealSendable (String moduleName, SwerveMotor swerveMotor, String motorName) {
        String name = moduleName + "." + motorName;
        Object realMotor = swerveMotor.getMotor();
        motors.put(name, realMotor);

        Sendable sendable = null;
        if (realMotor instanceof Sendable) {
            sendable = (Sendable) realMotor;
        } else {
            if (realMotor instanceof CANSparkMax) {
                sendable = new CANSparkMaxSendableWrapper((CANSparkMax) realMotor);
            }
        }
        if (sendable != null) {
            logger.info ("Adding sendable {}: {}", name, sendable);
            return new NS(name, sendable);
        }
        return null;
    }

    class NS {
        String n;
        Sendable s;
        NS (String _n, Sendable _s) {
            n = _n;
            s = _s;
        }
    }
}
