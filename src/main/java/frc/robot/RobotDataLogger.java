package frc.robot;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

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
	}
}