package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemLogging;

import static frc.robot.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase implements SubsystemLogging {
	private TalonSRX velocityTalon;
	private TalonSRX angleTalon;
	private CANSparkMax indexerNeo550;


	public ShooterSubsystem() {
		//TODO set Motor IDs in Constants
		this.velocityTalon = new TalonSRX(shooterVelocityTalonID);
		velocityTalon.setNeutralMode(NeutralMode.Brake);
		velocityTalon.configNeutralDeadband(0.001);


		this.angleTalon = new TalonSRX(shooterAngleTalonID);
		this.velocityTalon.setNeutralMode(NeutralMode.Brake);
		angleTalon.configNeutralDeadband(0.001);
		//init encoder to 0 position on boot
		angleTalon.setSelectedSensorPosition(0);


		this.indexerNeo550 = new CANSparkMax(shooterNeo550ID, CANSparkLowLevel.MotorType.kBrushless);
		indexerNeo550.setIdleMode(CANSparkBase.IdleMode.kBrake);

	}

	@Override
	public void periodic() {


		updateLogging();
	}


	public void updateLogging() {
		log("Shooter Velocity", getVelocityTalonVelocity());
		log("Shooter Angle Position", getAnglePosition());
		log("Shooter Angle Velocity")
	}

	/**
	 * Sets Velocity based on speed, in rpm
	 * Converts RPM to Sensor Units
	 * @param targetVelocity velocity
	 */
	public void setVelocityTalon(double targetVelocity) {
		velocityTalon.set(ControlMode.Velocity, targetVelocity * 4096 / 600);
	}

	/**
	 * Returns Velocity Talon's velocity in rpm
	 * @return ShooterVelocity
	 */
	public double getVelocityTalonVelocity() {
		return velocityTalon.getSelectedSensorVelocity() * 600 / 4096;
	}

	/**
	 * Sets the position of the Angle Talon Motor using Motion Magic
	 * TODO tune ff
	 * @param position encoder position
	 */
	public void setAngleTalonPosition(double position) {
		double feedForward = 0.00;
		angleTalon.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, feedForward);
	}


	/**
	 * Sets Angle Talon's Velocity in rpm (Shouldn't need to be used outside of testing)
	 * @param targetVelocity velocity
	 */
	public void setAngleTalonVelocity(double targetVelocity) {
		angleTalon.set(ControlMode.Velocity, targetVelocity * 4096 / 600);
	}

	public double getAnglePosition() {
		return angleTalon.getActiveTrajectoryPosition();
	}

	/**
	 * get Angle Talon velocity in rpm
	 * @return velocity rpm
	 */
	public double getAngleVelocity() {
		return angleTalon.getSelectedSensorVelocity() * 600 / 4096;
	}

	/**
	 * Sets Indexer Gearbox Neo550 Speed as a Percent [-1.0 - 1.0]
	 * @param speed percent velocoity
	 */
	public void setIndexerNeo550Speed(double speed) {
		indexerNeo550.set(speed);
	}






}
