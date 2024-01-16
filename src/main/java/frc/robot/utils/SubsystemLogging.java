package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public interface SubsystemLogging {
    default void log(String key, String val) {
        Logger.recordOutput(String.format("%s/%s", this.getClass().getName().substring(22), key), (val));    }

    default void log(String key, Object... val) {
        String className = this.getClass().getName();
        String simpleClassName = className.contains(".") ? className.substring(className.lastIndexOf('.') + 1) : className;
        String logKey = String.format("%s/%s", simpleClassName, key);
        
        for (Object value : val) {
            String logVal = value instanceof Object[] ? Arrays.deepToString((Object[]) value) : value.toString();

            Logger.recordOutput(logKey, logVal);
        }
    }
}
