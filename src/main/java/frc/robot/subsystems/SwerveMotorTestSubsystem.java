package frc.robot.subsystems;

import java.util.*;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CSPMXSW;
import org.usfirst.frc3620.misc.FakeMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;

/** Add your docs here. */
public class SwerveMotorTestSubsystem extends SubsystemBase {
    Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

    Map<String, Object> motors = new TreeMap<>();

    FakeMotor fm1, fm2;
    CANSparkMax csm;

    int i = 1000;

    public SwerveMotorTestSubsystem (SwerveDrive swerveDrive, int sm) {
        fm1 = new FakeMotor(4001);
        addChild ("fakeyfakey1", fm1);
        SwerveModule[] modules = swerveDrive.getModules();
        NS ns;
        for (var module : modules) {
            String moduleName = module.getConfiguration().name;
            ns = makeRealSendable(moduleName, module.getAngleMotor(), "angle");
            if (ns != null) addChild (ns.n, ns.s);
            ns = makeFakeSendable(moduleName, module.getAngleMotor(), "fakeangle");
            // if (ns != null) addChild (ns.n, ns.s);
            ns = makeRealSendable(moduleName, module.getDriveMotor(), "drive");
            if (ns != null) addChild (ns.n, ns.s);
            ns = makeFakeSendable(moduleName, module.getDriveMotor(), "fakedrive");
            if (ns != null) addChild (ns.n, ns.s);
        }
        logger.info(motors.toString());

        csm = new CANSparkMax(sm, MotorType.kBrushless);
        CSPMXSW smw = new CSPMXSW(csm);
        addChild("xfakeSparkMAX", smw);

        addChild("xfakeSparkMAXnull", null);

        fm2 = new FakeMotor(4002);
        addChild ("fakeyfakey2", fm2);
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
                sendable = new CSPMXSW((CANSparkMax) realMotor);
            }
        }
        if (sendable != null) {
            logger.info ("Adding sendable {}: {}", name, sendable);
            return new NS(name, sendable);
        }
        return null;
    }

    NS makeFakeSendable (String moduleName, SwerveMotor swerveMotor, String motorName) {
        String name = moduleName + "." + motorName;
        Sendable sendable = new FakeMotor(i+=100);
        logger.info ("Adding sendable {}: {}", name, sendable);
        return new NS("yyfake." + name, sendable);
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
