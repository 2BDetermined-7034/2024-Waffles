package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateDriveCommand extends Command {
    PIDController controller =  new PIDController(0,0,0);
   private SwerveSubsystem swerve;
    public RotateDriveCommand(SwerveSubsystem swerveSubsystem) {
      this.swerve = swerveSubsystem;

      controller.setTolerance(0.1);
    }

    @Override
    public void execute(){
        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
            swerve.drive(new Translation2d(0,0),
                    Rotation2d.fromRadians(controller.calculate(90.0,0.0)).getRadians(),
                    true);
        }
        else {

        }
    }
}
