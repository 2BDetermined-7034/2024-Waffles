// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */
	CANSparkMax motor1;
	CANSparkMax motor2;
	CANSparkMax motor3;
	public Intake() {
		motor1 = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless);
		motor2 = new CANSparkMax(11,CANSparkLowLevel.MotorType.kBrushless);
		motor2.setInverted(true);
		motor3 = new CANSparkMax(12,CANSparkLowLevel.MotorType.kBrushless);
		motor3.setInverted(true);

	}
   public void IntakeSetSpeed(double speed){
		motor1.set(speed);
	    motor2.set(speed);
	    motor3.set(speed);
   }
	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
				});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
