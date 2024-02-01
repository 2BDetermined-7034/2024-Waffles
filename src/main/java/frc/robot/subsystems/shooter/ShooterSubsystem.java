package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemLogging;

import static frc.robot.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase implements SubsystemLogging {
	private final TalonFX velocityTalon;
	private final TalonFX angleTalon;
	private final CANSparkMax indexerNeo550;

	/**
	 * Shooter Subsystem for Waffles Consisting of a Velocity Talon,
	 * Angle Adjustment Talon, and an Indexing Neo550
	 */
	public ShooterSubsystem() {
		//TODO set Motor IDs in Constants
		this.velocityTalon = new TalonFX(shooterVelocityTalonID);
		this.angleTalon = new TalonFX(shooterAngleTalonID);
		velocityTalon.setNeutralMode(NeutralModeValue.Brake);
		angleTalon.setNeutralMode(NeutralModeValue.Brake);

		//velocityTalon.configNeutralDeadband(0.001);

//		this.angleTalon = new TalonSRX(shooterAngleTalonID);
//		this.velocityTalon.setNeutralMode(NeutralMode.Brake);
//		angleTalon.configNeutralDeadband(0.001);
//		//init encoder to 0 position on boot
//		angleTalon.setSelectedSensorPosition(0);

		this.indexerNeo550 = new CANSparkMax(shooterNeo550ID, CANSparkLowLevel.MotorType.kBrushless);
		indexerNeo550.setIdleMode(CANSparkBase.IdleMode.kBrake);
	}

	@Override
	public void periodic() {


		updateLogging();
	}


	public void updateLogging() {

	}



	/**
	 * Sets Velocity based on speed [-1.0, 1.0]
	 * @param targetVelocity velocity
	 */
	public void setVelocityTalon(double targetVelocity) {
		velocityTalon.set(targetVelocity);
	}

	/**
	 * Returns Velocity Talon's velocity in rpm
	 * @return ShooterVelocity
	 */
	public double getVelocityTalonVelocity() {
		return velocityTalon.getVelocity().getValue();
	}

	/**
	 * Sets the position of the Angle Talon Motor in rotations
	 * TODO tune ff
	 * @param position encoder position
	 */
	public void setAngleTalonPosition(double position) {
		double feedForward = 0.00;
		angleTalon.setPosition(position);
	}


	/**
	 * Sets Angle Talon's Velocity in rpm
	 * @param targetVelocity velocity
	 */
	public void setAngleTalonVelocity(double targetVelocity) {
		angleTalon.set(targetVelocity);
	}

	public double getAnglePosition() {
		return angleTalon.getPosition().getValue();
	}

	/**
	 * get Angle Talon velocity in rpm
	 * @return velocity rpm
	 */
	public double getAngleVelocity() {
		return angleTalon.getVelocity().getValue();
	}

	/**
	 * Sets Indexer Gearbox Neo550 Speed as a Percent [-1.0 - 1.0]
	 * @param speed percent velocity
	 */
	public void setIndexerNeo550Speed(double speed) {
		indexerNeo550.set(speed);
	}
}
