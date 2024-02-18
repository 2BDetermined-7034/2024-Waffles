package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;
import frc.robot.utils.SubsystemLogging;
import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class ShooterCommand extends Command implements SubsystemLogging {
	protected ShooterSubsystem shooter;
	protected SwerveSubsystem swerveSubsystem;
	protected Photonvision photon = RobotContainer.photon;



	public ShooterCommand(ShooterSubsystem shooter, SwerveSubsystem swerveDrive) {
		this.shooter = shooter;
		this.swerveSubsystem = swerveDrive;
		addRequirements(shooter);
	}

	@Deprecated
	private double getDistanceToAngle(double distance) {
		return Math.atan((80.57-23.365)/(27+distance));
	}

	@Override
	public void execute() {
		//shooter.setVelocityTalon(0.3);
		//shooter.setAngleTalonVelocity(0.05);
		int tagID = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 7 : 4;


		//gets targets id 7, or 15 for speaker distance
		List<PhotonTrackedTarget> speakerTargetList = photon.targets().stream().filter((target) -> target.getFiducialId() == (tagID)).toList();
		if(!speakerTargetList.isEmpty()) {

			//shooter.setAngleFromTag(swerveSubsystem.getPose().getTranslation().minus(Constants.AprilTags.layout.get(tagID).pose.getTranslation().toTranslation2d()));
			PhotonTrackedTarget target = speakerTargetList.get(0);
			shooter.setAngleFromTag(target.getBestCameraToTarget().getTranslation().toTranslation2d());
		}
//		shooter.setAngleTalonPosition(1.0);
		shooter.setVelocityTalon(0.8);
		if(shooter.getVelocityTalonVelocity() >= 70){
			shooter.setIndexerNeo550Speed(0.6);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setVelocityTalon(0);
		shooter.setIndexerNeo550Speed(0);
	}
}
