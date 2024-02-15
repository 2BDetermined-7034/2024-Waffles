package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
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
		log("Angle Talon Goal", angleMotorPosition);
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
	 * Sets the position of the Angle Talon Motor in rotations<br>
	 * NOTE: This is in motor rotations, which,
	 * due to the magic of gears are not equivalent to a rotation of the shooter installation.<br>
	 * <b><i>You should probably use setPositionDegrees instead.</i></b>
	 * @param position encoder position
	 */
	public void setPosition(double position) {
		angleMotorPosition = MathUtil.clamp(position, 0, 3.5);
	}

	/**
	 * Sets the position of the angle Falcon in degrees
	 * (relative to the horizon, positive to tilt the shooter back relative to the horizon,
	 * negative to tilt the shooter forwards relative to the horizon).
	 * @param degrees
	 */
	public void setPositionDegrees(double degrees){
		setPosition(degreesToRotations(degrees));
	}

	/**
	 * <b><i>ONLY WORKS WITHIN 6 FEET (1.8 METERS)</i></b><br>
	 * Uses an equation to calculate the necessary angle to shoot given distance from the shooter.
	 * @param distance Distance (in meters) from the speaker
	 * @return Angle to shoot into the shooter at (in degrees)
	 */
	public double distanceToAngleWithin(double distance) {
		if(distance*39.37>72) {
			return Math.toDegrees(Math.atan((2.046 - (0.457 + 0.114 * Math.sin(0.423480295541))) / distance));
		}else{
			return Math.pow(distance, 0.9917707)*59.67448;
		}

		/*
		Here's what all the stupid numbers mean:
		Numerator: 2.046: height of the speaker (meters)
				   0.457: height of the shooter (meters)
				   0.114: distance from measuring point (axle of talon) to shooter exit (between the two rollers) (meters)
				   0.4234: average angle between the horizon and back-most point (in radians)
				           together with Math.sin() and the radius it creates an estimate for the height offset caused by rotation
		Denominator: distance from the point between the two rollers to the middle of the speaker
		 */
	}

	/**
	 * THIS DOES NOT WORK!!!
	 * TODO Fix scaling and get actual relative offset aside from 1.0 for zed.
	 * @param relativeTagPosition
	 */
	public void setAngleFromTag(Translation3d relativeTagPosition) {
		//TODO Figure out the distance from the target vertex to the tag
		double distance = relativeTagPosition.toTranslation2d().getNorm();
		log("Distance", distance);

		//Yes, min is greater than max. min is the base measurement
		// I have no idea what this is for so I'm, uhh, not gonna use it
//		final double minAngle = Math.toRadians(51.8);
//		final double maxAngle = Math.toRadians(32.1);

		/*
		double angle = (minAngle - Math.atan(target.getZ() / Math.abs(target.getX())));// / Math.PI / 2.0;
		log("Radian Target angle", angle);
		double scale = 3.5 / (minAngle - maxAngle);
		setPosition(angle * scale);
		 */

		setPositionDegrees(MathUtil.clamp(distanceToAngleWithin(distance), -28, 47));
		log("Distance To Angle", distanceToAngleWithin(distance));
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

	public double rotationsToDegrees(double rotations){
		return (((rotations / angleGearRatio) * 360.0) - angleAtHorizon) * -1;
	}
	public double degreesToRotations(double degrees){
		return ((-degrees + angleAtHorizon) / 360.0) * angleGearRatio;
	}
}
