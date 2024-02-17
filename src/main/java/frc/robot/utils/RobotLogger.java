package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;


public interface RobotLogger {
	default void log(String key, Pose2d val) {
		Logger.recordOutput(key, val);
	}
}
