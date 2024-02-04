package frc.robot.commands.shooter;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import static frc.robot.Constants.Shooter.*;

public class ShooterVision extends Command {
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
		new ShooterSubsystem().setAngleTalonPosition(getEquationValue(0)); //it's 0 because we're finding the distance later.
	}
}
