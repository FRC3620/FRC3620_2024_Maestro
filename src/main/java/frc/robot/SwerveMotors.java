// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.*;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANSparkMaxSendableWrapper;
import org.usfirst.frc3620.misc.FakeMotor;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;

/** Add your docs here. */
public class SwerveMotors {
    Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

    Map<String, Object> motors = new TreeMap<>();

    int i = 0;

    public SwerveMotors (SwerveSubsystem swerveSubsystem, SwerveDrive swerveDrive) {
        SwerveModule[] modules = swerveDrive.getModules();
        for (var module : modules) {
            String moduleName = module.getConfiguration().name;
            addMotor(swerveSubsystem, moduleName, module.getAngleMotor(), "angle");
            addMotor(swerveSubsystem, moduleName, module.getDriveMotor(), "drive");
        }
        logger.info(motors.toString());
    }

    void addMotor (SwerveSubsystem swerveSubsystem, String moduleName, SwerveMotor swerveMotor, String motorName) {
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
            swerveSubsystem.addChild (name, sendable);
        }
        /*
        swerveSubsystem.addChild("" + i + "xxx" + i, new FakeMotor(i));
        i++;
        */
    }
}
