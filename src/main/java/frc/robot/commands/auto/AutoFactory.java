package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;
import swervelib.SwerveDrive;

import javax.swing.text.html.Option;
import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Path;
import java.sql.Driver;
import java.util.ArrayList;
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
            Optional<Pose3d> tagOptional = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(aprilTagIndex);
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
        } catch (Exception e) {
            DriverStation.reportError("The tag map failed to load from file", true);
            return null;
        }
    }

    /**
     * Creates a 2m drive forward pathplanner autonomous command
     *
     * @return 2mStraight auto command
     */
    public static Command forward() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("2mStraight");
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

    /**
     * wtf did I make
     * @param swerve
     * @return
     */
    public static Command pathToSubwooferTrajectory(SwerveSubsystem swerve) {
        try {
            Optional<Pose3d> tagOptional = Optional.empty();
            switch (DriverStation.getAlliance().get()) {
                case Blue -> {
                    tagOptional = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(7);
                }
                case Red -> {
                    tagOptional = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile).getTagPose(3);
                }
            }

            if (tagOptional.isPresent()) {
                Pose3d tagPose = tagOptional.get();
                Pose2d currentPose = swerve.getPose();
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                        currentPose,
                        List.of(),
                        new Pose2d(
                                new Translation2d(tagPose.getX(), tagPose.getY()),
                                tagPose.getRotation().toRotation2d()
                        ),
                        new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2))
                                .setReversed(false)
                                .addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(2)))
                                .setKinematics(swerve.getKinematics())
                );
                return null;
            }
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
        return new WaitCommand(0);
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
                1,1,
                Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                3.0
        );
    }

    public static Command SubwooferAmp() {
        return pathFindThenFollowPath("Amp-Side-To-Subwoofer");
    }
    public static Command SubwooferSource() {
        return pathFindThenFollowPath("Source-Side-To-Subwoofer");
    }


}