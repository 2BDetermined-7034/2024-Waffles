package frc.robot.utils;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.Supplier;

public class Matrix {
    public double[][] data;
    int rows, columns;
    public Matrix() {}

    public Matrix(int rows, int columns) {
        data = new double[rows][columns];
        this.rows = rows;
        this.columns = columns;
    }

    public Matrix(double[][] data) {
        this.data = data;
    }

    public Matrix(Matrix matrix) {
        this.data = matrix.data;
    }

    public Matrix add(Matrix matrix) {
        if (rows != matrix.rows || columns != matrix.columns) {
            return null;
        }

        Matrix result = new Matrix(matrix.rows, columns);

        for (int row = 0; row < rows; ++row) {
            for (int column = 0; column < columns; ++columns) {
                result.data[row][column] = data[row][column] + matrix.data[row][column];
            }
        }

        return result;
    }

    public Matrix mul(Matrix matrix) {
        if (columns != matrix.rows) {
            return null;
        }

        Matrix result = new Matrix(matrix.rows, columns);

        for (int row = 0; row < rows; ++row) {
            for (int column = 0; column < columns; ++columns) {
                double sum = 0.0;
                for (int i = 0; i < columns; ++i) {
                    sum += data[row][i] * matrix.data[i][column];
                }

                result.data[row][column] = sum;
            }
        }

        return result;
    }

    public void translate(Translation3d translation) {
        if (rows != 4 && columns != 4) {
            return;
        }

//        data[0][3]
    }

    public static Pose3d toTransformations(Matrix matrix) {
        if (matrix.rows != 4 && matrix.columns != 4) {
            return null;
        }

        //All positions and rotations could potentially be inverted (Maybe not pitch?)
        double px, py, pz;
        double ry;

        px = matrix.data[0][3];
        py = matrix.data[1][3];
        pz = matrix.data[2][3];

        ry = Math.atan2(-matrix.data[0][1], matrix.data[0][0]);

        Translation3d position = new Translation3d(px, py, pz);
        Rotation3d orientation = new Rotation3d(0, 0, ry);
        return new Pose3d(position, orientation);
    }
/**
 * ID 1:
 * x = 15.07-16.45
 * y=0.24-8.2
 * z=1.35
 * yaw = 2.09 rad
 */

}
