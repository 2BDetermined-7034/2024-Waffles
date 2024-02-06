package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Climb.*;

/*
 *	Climbing Subsystem :
 * 	Motors		|	Function
 * 	-----------------------------
 * 	motorA	 	|	Runs on a PID to reach the target position
 * 	motorB		|	Follows motorA
 */
public class ClimbSubsystem extends SubsystemBase {
	protected CANSparkMax motorA, motorB;
	protected PIDController controller;
	protected RelativeEncoder encoderA, encoderB;
	public double targetPosition;

	public ClimbSubsystem() {
		motorA = new CANSparkMax(motorAID, CANSparkLowLevel.MotorType.kBrushless);
		motorB = new CANSparkMax(motorBID, CANSparkLowLevel.MotorType.kBrushless);

		final boolean swapInverted = true; //Modify if motors are running in the wrong direction
		motorA.setInverted(swapInverted);
		motorB.setInverted(!swapInverted);

		encoderA = motorA.getEncoder();
		encoderB = motorB.getEncoder();

		encoderA.setPosition(0.0);
		encoderB.setPosition(0.0);

		controller = new PIDController(0.5, 0.0, 0.3);
		controller.setTolerance(0.1);

		targetPosition = CLIMB_MOTOR_START_POSITION_REVOLUTIONS;
	}

	//Useless
	@Override
	public void periodic() {
		if (!controller.atSetpoint())
			motorA.set(controller.calculate(encoderA.getPosition(), targetPosition));
	}

	//Extend the climbing arm
	public void extend() {
		targetPosition = CLIMB_MOTOR_TARGET_POSITION_REVOLUTIONS;
	}

	//Retract the climbing arm
	public void retract() {
		targetPosition = CLIMB_MOTOR_START_POSITION_REVOLUTIONS;
	}

	public boolean isFinished() {
		return controller.atSetpoint();
	}
}