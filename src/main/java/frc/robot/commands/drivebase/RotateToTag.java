package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.SubsystemLogging;

public class RotateToTag extends Command implements SubsystemLogging {
	PIDController controller =  new PIDController(0.5,0.1,0);
	private SwerveSubsystem swerve;
	public RotateToTag(SwerveSubsystem swerveSubsystem) {
		this.swerve = swerveSubsystem;
		controller.enableContinuousInput(0,2 * Math.PI);
		controller.setTolerance(0.01, 0.1);
		addRequirements(swerveSubsystem);
	}

	@Override
	public void execute() {
//		if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){
//			swerve.drive(new Translation2d(0,0),
//					Rotation2d.fromRadians(controller.calculate(swerve.getYaw().getRadians(),
//							Constants.AprilTags.layout.get(4).pose.getRotation().getZ())).getRadians(),
//					true);
//		} else {
//			swerve.drive(new Translation2d(0,0),
//					Rotation2d.fromRadians(controller.calculate(swerve.getYaw().getRadians(),
//							Constants.AprilTags.layout.get(7).pose.getRotation().getZ())).getRadians(),
//					true);
//		}


		int tagID = DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red ? 4 : 7;
		Pose3d tagPose = Constants.AprilTags.layout.get(tagID).pose;
		Rotation2d rotationSetpoint = Rotation2d.fromRadians(Math.atan2(tagPose.getY() - swerve.getPose().getY(), tagPose.getX() - swerve.getPose().getX())).rotateBy(Rotation2d.fromDegrees(180));
		//double rotation  = Constants.AprilTags.layout.get(tagID).pose.getRotation().toRotation2d().getRadians() - swerve.getHeading().getRadians();
		log("Robot rotation setpoint", rotationSetpoint);
		log("Robot rotation error", controller.calculate(swerve.getHeading().getRadians(), rotationSetpoint.getRadians()));
		log("Robot rotation", swerve.getPose().getRotation());
		swerve.drive(new Translation2d(), controller.calculate(swerve.getHeading().getRadians(), rotationSetpoint.getRadians()) , true);
	}

	@Override
	public boolean isFinished() {
		return controller.atSetpoint();
	}
}
