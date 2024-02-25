package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemLogging;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Photonvision extends SubsystemBase implements SubsystemLogging {
    static boolean enablePhotonInstances = true; //Nyahaha
    private final PhotonCamera camera;
    private PhotonPipelineResult pipelineResult;

    public Photonvision() {
        camera = new PhotonCamera("greencase");
        pipelineResult = new PhotonPipelineResult();
    }

    public PhotonCamera getCamera() {
        return camera;
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
        pipelineResult = camera.getLatestResult();

    }

    public static void enableVision(boolean enable) {
        enablePhotonInstances = enable;
    }

}