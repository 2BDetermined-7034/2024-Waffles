package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
	protected NetworkTable networkTable;
	protected NetworkTableEntry tx, ty, ta;
	protected double x, y, area;
	protected final double errorCode = 0.0;

    Limelight() {
		networkTable = NetworkTableInstance.getDefault().getTable("limelight");
		tx = networkTable.getEntry("tx");
		ty = networkTable.getEntry("ty");
		ta = networkTable.getEntry("ta");
	}

    @Override
	public void periodic() {
		x = tx.getDouble(errorCode);
		y = ty.getDouble(errorCode);
		area = ta.getDouble(errorCode);

//		SmartDashboard.putNumber("Limelight X", x);
//		SmartDashboard.putNumber("Limelight Y", y);
//		SmartDashboard.putNumber("Limelight Area", area);
	}

	public double getX() { return x; }
	public double getY() { return y; }
	public double getArea() { return area; }
}
