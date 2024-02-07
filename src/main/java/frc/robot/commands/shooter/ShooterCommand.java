package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Photonvision;
import frc.robot.utils.SubsystemLogging;
import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class ShooterCommand extends Command implements SubsystemLogging {
	private ShooterSubsystem shooter;
	private Photonvision photon = RobotContainer.photon;

	public ShooterCommand(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	public double getDistanceToAngle(double distance) {
		return Math.atan((80.57-23.365)/(27+distance));
	}

	@Override
	public void execute() {
		//shooter.setVelocityTalon(0.3);
		//shooter.setAngleTalonVelocity(0.05);

		//gets targets id 7, or 15 for speaker distance
		List<PhotonTrackedTarget> speakerTargetList = photon.targets().stream().filter((target) -> target.getFiducialId() == 5 || target.getFiducialId() == 15).toList();
		double distance = -1; // default distance for no speaker target
		if(!speakerTargetList.isEmpty()) {
			distance = speakerTargetList.get(0).getBestCameraToTarget().getTranslation().getNorm();

			shooter.setAngleFromTag(speakerTargetList.get(0).getBestCameraToTarget().getTranslation());
		}
		log("Shooter Distance", distance);

		log("Shooter Angle", shooter.angleMotorPosition);
//		shooter.setAngleTalonPosition(1.0);
//		shooter.setVelocityTalon(0.4);
//		shooter.setIndexerNeo550Speed(0.6);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setVelocityTalon(0);
		shooter.setAngleTalonVelocity(0);
		shooter.setIndexerNeo550Speed(0);
	}
}
