package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import javax.swing.plaf.basic.BasicTabbedPaneUI;

import static frc.robot.Constants.Climb.*;

/*
 *	Climbing Subsystem :
 * 	Motors		|	Function
 * 	-----------------------------
 * 	motorA	 	|	Runs on a PID to reach the target position
 * 	motorB		|	Follows motorA
 */
public class ClimbSubsystem extends SubsystemBase {
	protected TalonFX motorA, motorB;
	protected PIDController controller;
	public double targetPosition;

	public ClimbSubsystem() {
		motorA = new TalonFX(motorAID);
		motorB = new TalonFX(motorBID);

		final boolean swapInverted = true; //Modify if motors are running in the wrong direction
		motorA.setInverted(swapInverted);
		motorB.setInverted(swapInverted);

		controller = new PIDController(0.5, 0.0, 0.0);
		controller.setTolerance(0.1);

		targetPosition = CLIMB_MOTOR_START_POSITION_REVOLUTIONS;
	}

	//Useless
	@Override
	public void periodic() {
		if (!controller.atSetpoint())
			motorA.set(controller.calculate(motorA.getPosition().getValue(), targetPosition));
	}

	public double getRevToDistance(double climbStroke){
		return ((Math.sqrt(Math.PI * Math.pow(SPOOL_RADIUS, 2) / Math.PI) - SPOOL_RADIUS) / BELT_THICKNESS) * CLIMB_MOTOR_GEAR_RATIO;
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