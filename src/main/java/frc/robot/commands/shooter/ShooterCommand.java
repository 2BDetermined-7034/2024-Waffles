package frc.robot.commands.shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
	private ShooterSubsystem shooter;
	private final PIDController angleController = new PIDController(0.4,0,0);
	public ShooterCommand(ShooterSubsystem shooter) {
		this.shooter = shooter;

		angleController.setTolerance(0.05);

		addRequirements(shooter);
	}
	/**Gets the equation of the hypotenuse of a triangle for the purpose of aiming based on trigonometry.
	 *<p>
	 *Equation = arctan((80.57-h)/(27+d))
	 *
	 *
	 * */
	public double getDistanceToAngle(double distance) {
		return Math.atan(57.205/(27+distance));
	}

	@Override
	public void execute() {
		//new ShooterSubsystem().setAngleTalonPosition(getEquationValue(0)); //TODO: find the distance and replace the parameter with it.
		//shooter.setVelocityTalon(0.8);
		//shooter.setIndexerNeo550Speed(0.3);
		shooter.setAngleTalonVelocity(0.05);

	}

	@Override
	public boolean isFinished() {
		return angleController.atSetpoint();
	}
	@Override
	public void end(boolean interrupted) {
		shooter.setVelocityTalon(0);
		shooter.setAngleTalonVelocity(0);
		shooter.setIndexerNeo550Speed(0);
	}
}
