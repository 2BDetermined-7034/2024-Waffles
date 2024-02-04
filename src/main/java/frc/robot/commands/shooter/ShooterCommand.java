package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
	private ShooterSubsystem shooterSubsystem;
	public ShooterCommand(ShooterSubsystem shooter) {
		this.shooterSubsystem = shooter;
		addRequirements(shooter);
	}
	/**Gets the equation of the hypotenuse of a triangle for the purpose of aiming based on trigonometry.
	 *<p>
	 *Equation = arctan((80.57-h)/(27+d))
	 *
	 *
	 * */
	public double getEquationValue(double distance) {
		return Math.atan(57.205/(27+distance));
	}

	@Override
	public void execute() {
		new ShooterSubsystem().setAngleTalonPosition(getEquationValue(0)); //TODO: find the distance and replace the parameter with it.
	}
}
