// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;


public class RobotContainer {
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private SlewRateLimiter m_Xfilter = new SlewRateLimiter(2);
  private SlewRateLimiter m_Yfilter = new SlewRateLimiter(2);


  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  XboxController operatorStick = new XboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(modifyXInput(joystick.getLeftY(), 0.05) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(modifyYInput(joystick.getLeftX(), 0.05) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    //REV set the arm subsystem to run the "runAutomatic" function continuously when no other command
    // is running
    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));


    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private Command runAuto = drivetrain.getAutoPath("StraightOnly");

  public RobotContainer() {
    configureBindings();
    configureButtonBindings();
    
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(() -> m_launcher.runLauncher(), m_launcher)
      .andThen(new WaitCommand(2))  //.andThen(m_launcher.stopLauncher(), m_launcher)
      .andThen(m_intake.feedLauncher(m_launcher));
    //return new WaitCommand(1.0);
    //return runAuto;
  }

  private void configureButtonBindings() {
    //Left Bumper Set Arm Scoring Position
    new JoystickButton(operatorStick, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));

    //Right Trigger Run Intake
    new Trigger( () -> operatorStick.getRightTriggerAxis() > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
        .onFalse(m_intake.retract());

    //Left Trigger Set Arm Intake Position
    new Trigger(() -> operatorStick.getLeftTriggerAxis() > Constants.OIConstants.kTriggerButtonThreshold)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));

      //Start Button Set Arm Home Position
      new JoystickButton(operatorStick, XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

    //X Button Reverse Launcher
    new JoystickButton(operatorStick, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> m_launcher.reverseLauncher()));

    new JoystickButton(operatorStick, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(operatorStick, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    new JoystickButton(operatorStick, XboxController.Button.kA.value)
        .onTrue(m_intake.feedLauncher(m_launcher));
  }

  private double modifyXInput(double _value, double _deadband){
    double x_value = m_Xfilter.calculate(_value);
    return MathUtil.applyDeadband(x_value, _deadband);
  }

  private double modifyYInput(double _value, double _deadband){
    double y_value = m_Yfilter.calculate(_value);
    return MathUtil.applyDeadband(y_value, _deadband);  
  }

}
