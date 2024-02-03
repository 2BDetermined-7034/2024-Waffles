package frc.robot.commands.swervedrive.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbExtendCommand extends Command {
	protected ClimbSubsystem climb;

	public ClimbExtendCommand(ClimbSubsystem climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	@Override
	public void initialize() {
		climb.extend();
	}

	@Override
	public boolean isFinished() {
		return climb.isFinished();
	}
}
