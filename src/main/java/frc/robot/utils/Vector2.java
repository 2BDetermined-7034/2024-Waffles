package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vector2 {
    public double x, y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(double s) {
        this.x = s;
        this.y = s;
    }

	public Vector2(Translation2d translation2d) {
		this.x = translation2d.getX();
		this.y = translation2d.getY();
	}

	public Vector2(Transform2d transform2d) {
		this.x = transform2d.getX();
		this.y = transform2d.getY();
	}

	public Vector2(Pose2d pose2d) {
		this.x = pose2d.getX();
		this.y = pose2d.getY();
	}

    public Vector2 add(Vector2 vector) { return new Vector2(this.x + vector.x, this.y + vector.y); }
	public Vector2 sub(Vector2 vector) { return new Vector2(this.x - vector.x, this.y - vector.y); }
	public Vector2 mul(Vector2 vector) { return new Vector2(this.x * vector.x, this.y * vector.y); }
	public Vector2 div(Vector2 vector) { return new Vector2(this.x / vector.x, this.y / vector.y); }
	public Vector2 mul(double s) { return new Vector2(this.x * s, this.y * s); }
	public Vector2 div(double s) { return new Vector2(this.x / s, this.y / s); }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public static double dot(Vector2 a, Vector2 b) {
        return a.x * b.x + a.y * b.y;
    }

	public Translation2d toTranslation2d() {
		return new Translation2d(x, y);
	}
}
