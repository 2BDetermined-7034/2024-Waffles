package frc.robot.commands.drivebase;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateDriveCommand extends Command {
	PIDController controller =  new PIDController(0,0,0);
	private SwerveSubsystem swerve;
	public RotateDriveCommand(SwerveSubsystem swerveSubsystem) {
		this.swerve = swerveSubsystem;

		controller.setTolerance(0.1);
	}

	@Override
	public void execute() {
		if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){
			swerve.drive(new Translation2d(0,0),
					Rotation2d.fromRadians(controller.calculate(swerve.getYaw().getRadians(),
							Constants.AprilTags.layout.get(4).pose.getRotation().getZ())).getRadians(),
					true);
		} else {
			swerve.drive(new Translation2d(0,0),
					Rotation2d.fromRadians(controller.calculate(swerve.getYaw().getRadians(),
							Constants.AprilTags.layout.get(7).pose.getRotation().getZ())).getRadians(),
					true);
		}
	}
}
