package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

import java.util.function.DoubleSupplier;

public class ControllerDrive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier rawAxis;
    private final boolean isOpenLoop;
    private final SwerveController controller;

    public ControllerDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rawAxis, boolean isOpenLoop) {
        this.swerveSubsystem = swerveSubsystem;
        this.x = x;
        this.y = y;
        this.rawAxis = rawAxis;
        this.isOpenLoop = isOpenLoop;
        this.controller = swerveSubsystem.getSwerveController();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(x.getAsDouble(), y.getAsDouble()), rawAxis.getAsDouble() , true);
        //swerveSubsystem.drive(new Translation2d(0, 0), 0, true, isOpenLoop);
        // REMEMBER - X (first variable) in the translation 2D corresponds to forwards movement!
        // "positive x is torwards the bow (front) and positive y is torwards port (left)" WHYYY
        // 💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀💀
    }


}