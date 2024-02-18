package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;

import java.util.List;
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
//        Photonvision.enableVision(false);
        PathConstraints constraints = new PathConstraints(
                1, 1,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        // .andThen(new InstantCommand(() -> Photonvision.enableVision(true)));
    }

    public static Command pointTowardsTag(int aprilTagIndex, SwerveSubsystem swerve) {
        Pose3d tagPosition;
        try {
            Optional<Pose3d> tagOptional =  AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(aprilTagIndex);
            if (tagOptional.isPresent()) {
                tagPosition = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(aprilTagIndex).get();

                double rotation = Math.atan((tagPosition.getX() - swerve.getPose().getX()) / (tagPosition.getY() - swerve.getPose().getY()));
                Pose2d targetPose = new Pose2d(new Translation2d(swerve.getPose().getX(), swerve.getPose().getY()), new Rotation2d(rotation));
//                Photonvision.enableVision(false);
                PathConstraints constraints = new PathConstraints(
                        0, 0,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));
                return AutoBuilder.pathfindToPose(
                        targetPose,
                        constraints,
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                );
                // .andThen(new InstantCommand(() -> Photonvision.enableVision(true)));
            } else {
//                DriverStation.reportError("The tag failed to load in: pointTowardsTag", true);
                return null;
            }
        } catch  (Exception e) {
//            DriverStation.reportError("The tag map failed to load from file", true);
            return null;
        }
    }

    public static Command forward() {
        PathPlannerPath path =  PathPlannerPath.fromPathFile("2mStraight");
        return AutoBuilder.followPath(path);

    }

    public static Command flyPathFindWithPoses() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        path.preventFlipping = true;
        return AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI));
    }


    public static Command pathFindThenFollowPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        PathConstraints constraints = new PathConstraints(
                5,8,
                Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                0
        );
    }

}
