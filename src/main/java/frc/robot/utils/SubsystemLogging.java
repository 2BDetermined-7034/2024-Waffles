package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.HashMap;

public interface SubsystemLogging {
    default void log(String key, String val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }


    default void log(String key, double val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }default void log(String key, int val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, double[] val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }

    default void log(String key, SwerveModuleState[] val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }
    default void log(String key, Pose2d val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }

    default void log(String key, Pose3d val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));
    }



//    default void log(String key, Object... val) {
//        String className = this.getClass().getName();
//        String simpleClassName = className.contains(".") ? className.substring(className.lastIndexOf('.') + 1) : className;
//        String logKey = String.format("%s/%s", simpleClassName, key);
//
//        for (Object value : val) {
//            String logVal = value instanceof Object[] ? Arrays.deepToString((Object[]) value) : value.toString();
//
//            Logger.recordOutput(logKey, logVal);
//        }
//    }
}
