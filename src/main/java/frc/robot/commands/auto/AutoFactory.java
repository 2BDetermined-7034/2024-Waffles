package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;

import java.util.Optional;

public class AutoFactory {
    public static Command driveUpToTarget() {
        Pose2d targetPose = new Pose2d(new Translation2d(2.2, 5), new Rotation2d(0));
        //Photonvision.enableVision(false);
        PathConstraints constraints = new PathConstraints(
                1, 1,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }
    public static Command rotate(SwerveSubsystem swerve) {
        Pose2d targetPose = new Pose2d(new Translation2d(swerve.getPose().getX(), swerve.getPose().getY()), new Rotation2d(Math.PI));
        Photonvision.enableVision(false);
        PathConstraints constraints = new PathConstraints(
                1, 1,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        ).andThen(new InstantCommand(() -> Photonvision.enableVision(true)));
    }

    public static Command pointTowardsTag(int aprilTagIndex, SwerveSubsystem swerve) {
        Pose3d tagPosition;
        try {
            Optional<Pose3d> tagOptional =  AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(aprilTagIndex);
            if (tagOptional.isPresent()) {
                tagPosition = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(aprilTagIndex).get();

                double rotation = Math.atan((tagPosition.getX() - swerve.getPose().getX()) / (tagPosition.getY() - swerve.getPose().getY()));
                Pose2d targetPose = new Pose2d(new Translation2d(swerve.getPose().getX(), swerve.getPose().getY()), Rotation2d.fromRadians(rotation));
                Photonvision.enableVision(false);
                PathConstraints constraints = new PathConstraints(
                        0, 0,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));
                return AutoBuilder.pathfindToPose(
                        targetPose,
                        constraints,
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                ).andThen(new InstantCommand(() -> Photonvision.enableVision(true)));
            } else {
                DriverStation.reportError("The tag failed to load in: pointTowardsTag", true);
                return null;
            }
        } catch  (Exception e) {
            DriverStation.reportError("The tag map failed to load from file", true);
            return null;
        }
    }

    public static Command forward() {
        PathPlannerPath path =  PathPlannerPath.fromPathFile("2mStraight");
        return AutoBuilder.followPath(path);

    }

    public static Command pointTowardsSpeakerTag(SwerveSubsystem swerveSubsystem) {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
                return pointTowardsTag(4, swerveSubsystem);
            else
                return pointTowardsTag(5, swerveSubsystem);
        }

        return new WaitCommand(0.0);
    }
}