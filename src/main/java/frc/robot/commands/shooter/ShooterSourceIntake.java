package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

public class ShooterSourceIntake extends ShooterCommand {


	public ShooterSourceIntake(ShooterSubsystem shooter, SwerveSubsystem swerveDrive) {
		super(shooter, swerveDrive);
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
		shooter.setPositionDegrees(-15);
//		shooter.setAngleTalonPosition(1.0);
		shooter.setVelocityTalon(0);
		shooter.setIndexerNeo550Speed(0.6);
	}
}
