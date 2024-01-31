package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
	private ShooterSubsystem shooter;

	public ShooterCommand(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.setVelocityTalon(250);
//		shooter.setAngleTalonPosition(1000);
		shooter.setAngleTalonVelocity(250);
		shooter.setIndexerNeo550Speed(0.2);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setVelocityTalon(0);
		shooter.setAngleTalonVelocity(0);
		shooter.setIndexerNeo550Speed(0);
	}
}
