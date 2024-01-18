package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Photonvision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPipelineResult pipelineResult;

    public Photonvision() {
        camera = new PhotonCamera("YourCameraName");
        pipelineResult = new PhotonPipelineResult();
    }

   public boolean hasTargets() {
        return pipelineResult.hasTargets();
   }

   public List<PhotonTrackedTarget> targets() {
        return pipelineResult.targets;
   }

   public PhotonTrackedTarget getBestTarget() {
        return pipelineResult.getBestTarget();
   }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
