package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemLogging;

import static frc.robot.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase implements SubsystemLogging {
	private final TalonFX velocityTalon;
	private final TalonFX angleTalon;
	private final CANSparkMax indexerNeo550;
	private final PositionVoltage anglePositionVoltage;

	public final double homeAngle = 0;
	public final double maxAngle = 0.2;
	public double angleMotorPosition = 0;


	/**
	 * Shooter Subsystem for Waffles Consisting of a Velocity Talon,
	 * Angle Adjustment Talon, and an Indexing Neo550
	 */
	public ShooterSubsystem() {
		//TODO set Motor IDs in Constants
		this.velocityTalon = new TalonFX(shooterVelocityTalonID);
		this.angleTalon = new TalonFX(shooterAngleTalonID);

		velocityTalon.setNeutralMode(NeutralModeValue.Coast);

		// Angle Talon
		angleTalon.setNeutralMode(NeutralModeValue.Brake);

		Slot0Configs angleMotorPID = new Slot0Configs();
		angleMotorPID.kP = 1;
		angleMotorPID.kI = 0;
		angleMotorPID.kD = 0;
		angleMotorPID.kS = 0.24;

		angleTalon.getConfigurator().apply(angleMotorPID);

		anglePositionVoltage = new PositionVoltage(0);

		// robot init, set slot 0 gains
		this.indexerNeo550 = new CANSparkMax(shooterNeo550ID, CANSparkLowLevel.MotorType.kBrushless);
		indexerNeo550.setIdleMode(CANSparkBase.IdleMode.kBrake);

		//init encoder to 0 position on boot
		angleTalon.setPosition(0);
	}

	@Override
	public void periodic() {
		anglePositionVoltage.Position = angleMotorPosition;
		angleTalon.setControl(anglePositionVoltage);

		updateLogging();
	}


	public void updateLogging() {
		log("Shooter Talon Velocity", getVelocityTalonVelocity());
		log("Angle Talon Velocity", getAngleVelocity());
		log("Angle Talon Position", getAnglePosition());
	}



	/**
	 * Sets Velocity based on speed [-1.0, 1.0]
	 * @param targetVelocity velocity
	 */
	public void setVelocityTalon(double targetVelocity) {
		velocityTalon.set(MathUtil.clamp(targetVelocity, -1, 1));
	}



	public void zeroAngleTalon() {
		// TODO implement if possible
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
	 *
	 * TODO tune ff
	 * @param position encoder position
	 */
	public void setAngleTalonPosition(double position) {
		angleMotorPosition = MathUtil.clamp(position, 0, 3.5);
	}


	/**
	 * Sets Angle Talon's Velocity as a percent[-1.0, 1.0]
	 * @param targetVelocity velocity
	 */
	public void setAngleTalonVelocity(double targetVelocity) {
		angleTalon.set(MathUtil.clamp(targetVelocity, -1, 1));
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
	public void setIndexerNeo550Speed(double speed) { indexerNeo550.set(MathUtil.clamp(speed, -1, 1)); }
}
