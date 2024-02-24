package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterManualAngle extends ShooterCommand {
	DoubleSupplier pos;
	DoubleSupplier neg;


	public ShooterManualAngle(ShooterSubsystem shooter, SwerveSubsystem swerveDrive, DoubleSupplier posAxis, DoubleSupplier negAxis) {
		super(shooter, swerveDrive);
		pos = posAxis;
		neg = negAxis;
	}

	@Override
	public void execute() {
		//shooter.setVelocityTalon(0.3);
		//shooter.setAngleTalonVelocity(0.05);

		//gets targets id 7, or 15 for speaker distance
//		List<PhotonTrackedTarget> speakerTargetList = photon.targets().stream().filter((target) -> target.getFiducialId() == (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 7 : 4)).toList();
//		if(!speakerTargetList.isEmpty()) {
//			shooter.setAngleFromTag(speakerTargetList.get(0).getBestCameraToTarget().getTranslation());
//		}
		shooter.setPositionDegrees(shooter.getAngleGoalDegrees() + (pos.getAsDouble() * 0.05) - (neg.getAsDouble() * 0.05));
//		shooter.setAngleTalonPosition(1.0);
		shooter.setVelocityTalon(0);
		shooter.setIndexerNeo550Speed(0.0);
	}
}