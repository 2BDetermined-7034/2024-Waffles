package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemLogging;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class Photonvision extends SubsystemBase implements SubsystemLogging {
    static boolean enablePhotonInstances = true; //Nyahaha
    private final PhotonPipelineResult pipelineResult;
    private PhotonPoseEstimator poseEstimator;

    public Photonvision(String cameraName, Transform3d cameraToRobot) {
        PhotonCamera camera = new PhotonCamera(cameraName);
        pipelineResult = new PhotonPipelineResult();

        try {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);
        } catch(IOException e) {
            DriverStation.reportError(e.toString(), true);
        }
    }


    /**
     *
     * @param prevEstimatedRobotPose Robot's current pose
     * @return The Estimated Robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    /**
     *
     * @return Boolean, if there's a target or not
     */
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


    }

    public static void enableVision(boolean enable) {
        enablePhotonInstances = enable;
    }
}
