package frc.robot;

import java.util.EnumSet;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.Utilities;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class RobotDataLogger {
	PowerDistribution powerDistribution = null;
	Runtime runtime = null;

	public RobotDataLogger (DataLogger dataLogger, CANDeviceFinder canDeviceFinder) {
		dataLogger.addDataProvider("matchTime", () -> DataLogger.f2(DriverStation.getMatchTime()));
		dataLogger.addDataProvider("robotMode", () -> Robot.getCurrentRobotMode().toString());
		dataLogger.addDataProvider("robotModeInt", () -> Robot.getCurrentRobotMode().ordinal());
		dataLogger.addDataProvider("batteryVoltage", () -> DataLogger.f2(RobotController.getBatteryVoltage()));

		powerDistribution = RobotContainer.powerDistribution;

		if (powerDistribution != null) {
			dataLogger.addDataProvider("pdp.totalCurrent", () -> DataLogger.f2(powerDistribution.getTotalCurrent()));
			dataLogger.addDataProvider("pdp.totalPower", () -> DataLogger.f2(powerDistribution.getTotalPower()));
			dataLogger.addDataProvider("pdp.totalEnergy", () -> DataLogger.f2(powerDistribution.getTotalEnergy()));
		}

		runtime = Runtime.getRuntime();

		dataLogger.addDataProvider ("mem.free", () -> runtime.freeMemory());
		dataLogger.addDataProvider ("mem.total", () -> runtime.totalMemory());
		dataLogger.addDataProvider ("mem.max", () -> runtime.maxMemory());

		addSwerveDataLoggers(dataLogger);
		
		// weAreAiming
		dataLogger.addDataProvider("weAreAiming",() -> RobotContainer.drivebase.getAreWeAiming());
		// motor temps
		if( RobotContainer.climbElevationSubsystem.motor != null){
		dataLogger.addDataProvider("climber.temperature",() ->  RobotContainer.climbElevationSubsystem.motor.getMotorTemperature());
		}		
		// we have a target?
		dataLogger.addDataProvider("doWeHaveATarget",() -> RobotContainer.visionSubsystem.doIHaveTarget());
		// requested vs actual headings
		// shooter motor velocities
		if(RobotContainer.shooterSubsystem.topMotor != null){
		dataLogger.addDataProvider("shooter.top.velocity",() -> RobotContainer.shooterSubsystem.topMotor.getVelocity());
		}
		if(RobotContainer.shooterSubsystem.bottomMotor != null){
		dataLogger.addDataProvider("shooter.top.velocity",() -> RobotContainer.shooterSubsystem.bottomMotor.getVelocity());
		}
	}

	void addSwerveDataLoggers(DataLogger dataLogger) {
        SwerveModule[] modules = RobotContainer.drivebase.getSwerveDrive().getModules();
        for (var module : modules) {
            String moduleName = "swerve." + module.getConfiguration().name;
			addMotorProviders (dataLogger, moduleName + ".angle", module.getAngleMotor().getMotor(), EnumSet.allOf(MotorFields.class));
			addMotorProviders (dataLogger, moduleName + ".drive", module.getDriveMotor().getMotor(), EnumSet.allOf(MotorFields.class));
        }
    }

	void addMotorProviders (DataLogger dataLogger, String name, Object motor) {
		addMotorProviders (dataLogger, name, motor, EnumSet.of(MotorFields.CURRENT, MotorFields.TEMPERATURE, MotorFields.OUTPUT));
	}

	void addMotorProviders (DataLogger dataLogger, String name, Object motor, EnumSet<MotorFields> fields) {
		dataLogger.addDataProvider (name + ".object", () -> motor.toString());
		if (motor instanceof CANSparkBase) {
			CANSparkBase m = (CANSparkBase) motor;
			if (fields.contains(MotorFields.TEMPERATURE))
				dataLogger.addDataProvider (name + ".temperature", () -> m.getMotorTemperature());
			if (fields.contains(MotorFields.CURRENT))
				dataLogger.addDataProvider (name + ".current", () -> m.getOutputCurrent());
			if (fields.contains(MotorFields.OUTPUT))
				dataLogger.addDataProvider (name + ".output", () -> m.getAppliedOutput());
		} else if (motor instanceof TalonFX) {
			TalonFX m = (TalonFX) motor;
			if (fields.contains(MotorFields.TEMPERATURE))
				dataLogger.addDataProvider (name + ".temperature", () -> m.getDeviceTemp());
			if (fields.contains(MotorFields.CURRENT))
				dataLogger.addDataProvider (name + ".current", () -> m.getStatorCurrent());
			if (fields.contains(MotorFields.OUTPUT))
				dataLogger.addDataProvider (name + ".output", () -> m.getDutyCycle().getValue());
		}
	}

	enum MotorFields {
		TEMPERATURE, CURRENT, OUTPUT
	}

}