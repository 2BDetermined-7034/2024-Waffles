package frc.robot.utils;

public class Deadbanding {
	public Vector2 applyDeadband(Vector2 input, double deadZone, double deadband) {
		double magnitude = input.magnitude();

		input.div(magnitude);

		if (magnitude < deadZone) return new Vector2(0.0);
		magnitude = Math.min(magnitude, 1.0);

		magnitude = deadband * (magnitude * magnitude * magnitude) + (1.0 - deadband) * magnitude;

		input.mul(magnitude);

		return input;
	}
}
