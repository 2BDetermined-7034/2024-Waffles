package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Photonvision;

public class PhotonTest extends Command {
    private final Photonvision photonvision;

    public PhotonTest(Photonvision photonvision) {
        this.photonvision = photonvision;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.photonvision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {


    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
