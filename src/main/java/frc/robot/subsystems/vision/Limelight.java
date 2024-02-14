package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SubsystemLogging;

public class Limelight extends SubsystemBase implements SubsystemLogging {
	protected NetworkTable networkTable;
	protected NetworkTableEntry tx, ty, ta, botpose;
	protected double x, y, area;
	protected final double errorCode = 0.0;
    public Limelight() {
		networkTable = NetworkTableInstance.getDefault().getTable("limelight");
		tx = networkTable.getEntry("tx");
		ty = networkTable.getEntry("ty");
		ta = networkTable.getEntry("ta");
		botpose = networkTable.getEntry("botpose_wpired");
	}

    @Override
	public void periodic() {
		x = tx.getDouble(errorCode);
		y = ty.getDouble(errorCode);
		area = ta.getDouble(errorCode);

		logger();

//		SmartDashboard.putNumber("Limelight X", x);
//		SmartDashboard.putNumber("Limelight Y", y);
//		SmartDashboard.putNumber("Limelight Area", area);
	}

	public double getX() { return x; }
	public double getY() { return y; }
	public double getArea() { return area; }
	public double[] getBotPose() {
		return botpose.getDoubleArray(new double[0]);
	}

	public Pose3d getBotPose3D() {
		return new Pose3d(new Translation3d(getBotPose()[0], getBotPose()[1], getBotPose()[2]), new Rotation3d(getBotPose()[3], getBotPose()[4], getBotPose()[5]));
	}



	public void logger() {
		log("X", getX());
		log("y", getY());
		log("Area", getArea());
		log("Botpose Double", getBotPose());
		log("Actual bot pose", getBotPose3D());
		//log("Tag 1 Poses", Constants.AprilTags.tagmap.get(1));
	}
}
