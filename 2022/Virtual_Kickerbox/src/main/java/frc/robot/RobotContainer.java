// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SpinMotor;
import frc.robot.commands.SpinMotorRev;
import frc.robot.subsystems.Kickerbox;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SpinMotor;
import frc.robot.commands.StopMotor;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Kickerbox m_Kickerbox = new Kickerbox();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController operator =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    operator.a().whileTrue(new SpinMotor(1));
    operator.a().and(operator.leftBumper()).whileTrue(new SpinMotorRev(1));
    operator.a().onFalse(new StopMotor(1));

    operator.b().whileTrue(new SpinMotor(2));
    operator.b().and(operator.leftBumper()).whileTrue(new SpinMotorRev(2));
    operator.b().onFalse(new StopMotor(2));

    operator.x().whileTrue(new SpinMotor(3));
    operator.x().and(operator.leftBumper()).whileTrue(new SpinMotorRev(3));
    operator.x().onFalse(new StopMotor(3));
  }
}
