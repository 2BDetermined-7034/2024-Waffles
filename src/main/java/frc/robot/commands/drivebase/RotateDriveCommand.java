package frc.robot.commands.drivebase;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.SubsystemLogging;

public class RotateDriveCommand extends Command implements SubsystemLogging {
	PIDController controller =  new PIDController(0.2,0,0);
	private SwerveSubsystem swerve;
	public RotateDriveCommand(SwerveSubsystem swerveSubsystem) {
		this.swerve = swerveSubsystem;
		controller.enableContinuousInput(-Math.PI,Math.PI);
		controller.setTolerance(0.1);
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


		int tagID = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red) ? 4 : 7;
		double rotation = Math.atan2(Constants.AprilTags.layout.get(tagID).pose.getY() - swerve.getPose().getY(), Constants.AprilTags.layout.get(tagID).pose.getX() - swerve.getPose().getX());
		//double rotation  = Constants.AprilTags.layout.get(tagID).pose.getRotation().toRotation2d().getRadians() - swerve.getHeading().getRadians();
		log("Shooter rotation  error", rotation);
		swerve.drive(new Translation2d(), controller.calculate(swerve.getHeading().getRadians(), rotation) , true);
	}

	@Override
	public boolean isFinished() {
		return controller.atSetpoint();

	}
}
