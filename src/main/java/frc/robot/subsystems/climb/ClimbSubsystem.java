package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
	CANSparkMax cwMotor, ccwMotor;

	public ClimbSubsystem() {
		cwMotor = new CANSparkMax(255, CANSparkLowLevel.MotorType.kBrushless);
		ccwMotor = new CANSparkMax(69, CANSparkLowLevel.MotorType.kBrushless);

		boolean swapInverted = false;
		ccwMotor.setInverted(swapInverted);
		cwMotor.setInverted(!swapInverted);
	}

	@Override
	public void periodic() {

	}

	public void setTargetPosition(double position) {
		cwMotor.getEncoder().setPosition(position);
		ccwMotor.getEncoder().setPosition(position);
	}

	public void erect() {
		setTargetPosition(Constants.CLIMB_MOTOR_TARGET_POSITION_REVOLUTIONS);
	}

	public void retract() {
		setTargetPosition(Constants.CLIMB_MOTOR_START_POSITION_REVOLUTIONS); //It's zero btw - Jack M >:[
	}
}



