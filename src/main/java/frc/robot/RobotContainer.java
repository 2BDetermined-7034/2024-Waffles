// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.drivebase.ControllerDrive;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;

  public static Photonvision photon = new Photonvision();



  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  XboxController operatorController = new XboxController(1);
  PS5Controller driverController = new PS5Controller(0);

//  public static Limelight limelight= new Limelight();


  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, drivebase);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("New Auto", new PathPlannerAuto("New Auto"));
    //autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);



    // Configure the trigger bindings
    configureBindings();

    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX(), () -> true);

    TeleopDrive closedFieldRelOperator = new TeleopDrive(
            drivebase,
            () -> MathUtil.applyDeadband(operatorController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(operatorController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),

            () -> operatorController.getRightX(), () -> true);

    drivebase.setDefaultCommand(new ControllerDrive(drivebase,
            () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driverController.getRawAxis(2), true));


    drivebase.setDefaultCommand(closedFieldRel);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */


  private void configureBindings()
  {
    //new JoystickButton(driverController, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
   // new Trigger(driverController::getCircleButton).toggleOnTrue(new AutoFactory().driveUpToTarget());

    //new Trigger(() -> operatorController.getBackButton()).onTrue(new InstantCommand(drivebase::zeroGyro));
    new Trigger(driverController::getOptionsButton).onTrue(new InstantCommand(drivebase::zeroGyro));

    new Trigger(driverController::getCircleButton).toggleOnTrue(shooterCommand);

    new Trigger(driverController::getL2Button).onTrue(AutoFactory.pointTowardsSpeakerTag(drivebase));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Path", true);
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setDriveMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
