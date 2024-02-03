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
* 	cwMotor 	|	Clockwise motor for one of the coils
* 	ccwMotor	|	Counter-Clockwise motor for one of the coils
*
* 	NOTE:
* 	The actual direction of the motor doesn't matter, as long as they are spinning in opposite directions.
*/
public class ClimbSubsystem extends SubsystemBase {
	protected CANSparkMax cwMotor, ccwMotor;
	protected PIDController cwController, ccwController;
	protected RelativeEncoder cwEncoder, ccwEncoder;
	public double targetPosition;

	public ClimbSubsystem() {
		cwMotor = new CANSparkMax(cwMotorID, CANSparkLowLevel.MotorType.kBrushless);
		ccwMotor = new CANSparkMax(ccwMotorID, CANSparkLowLevel.MotorType.kBrushless);

		final boolean swapInverted = true; //Modify if motors are running in the wrong direction
		cwMotor.setInverted(swapInverted);
		ccwMotor.setInverted(!swapInverted);

		cwEncoder = cwMotor.getEncoder();
		ccwEncoder = ccwMotor.getEncoder();

		cwEncoder.setPosition(0.0);
		ccwEncoder.setPosition(0.0);

		cwController = new PIDController(0.5, 0.0, 0.3);
		ccwController = new PIDController(0.5, 0.0, 0.3);

		cwController.setTolerance(0.1);
		ccwController.setTolerance(0.1);

		targetPosition = CLIMB_MOTOR_START_POSITION_REVOLUTIONS;
	}

	//Useless
	@Override
	public void periodic() {
		if (!cwController.atSetpoint())
			cwMotor.set(cwController.calculate(cwEncoder.getPosition(), targetPosition));
		if (!ccwController.atSetpoint())
			ccwMotor.set(ccwController.calculate(ccwEncoder.getPosition(), targetPosition));
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
		return cwController.atSetpoint() && ccwController.atSetpoint();
	}
}



