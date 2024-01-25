// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private Drivetrain m_Drivetrain;
  private Controller m_Controller;

  private SendableChooser<String> driveModeChooser;
  private SendableChooser<Double> controlChoiceChooser;
  private String d_a = Constants.d_a;
  private String d_t = Constants.d_t;

  @Override
  public void robotInit() {
    m_Drivetrain = new Drivetrain();
    m_Controller = new Controller();
    m_Controller.setDrivetrainReference(m_Drivetrain);
    m_Drivetrain.setControllerReference(m_Controller);

    driveModeChooser = new SendableChooser<String>();
    controlChoiceChooser = new SendableChooser<Double>();
    driveModeChooser.setDefaultOption("Arcade Drive", d_a);
    driveModeChooser.addOption("Tank Drive", d_t);
    SmartDashboard.putData("Drive Mode", driveModeChooser);

    // Add controller preset indeces as doubles here -- reference the index of the preset values set and then the index 0 OF THAT
    // LIST to add the reference index as the option
    controlChoiceChooser.addOption("Joystick General", Constants.controllerChoices[0][0]);
    controlChoiceChooser.setDefaultOption("Joystick Scaled Up (L2 & R4)", Constants.controllerChoices[2][0]);
    controlChoiceChooser.addOption("Joystick Hyper (L2 & R4)", Constants.controllerChoices[4][0]);
    controlChoiceChooser.addOption("Xbox General", Constants.controllerChoices[1][0]);
    controlChoiceChooser.addOption("Xbox X3", Constants.controllerChoices[3][0]);
    controlChoiceChooser.addOption("Xbox X3 Hyper", Constants.controllerChoices[5][0]);


    SmartDashboard.putData("Control Choice", controlChoiceChooser);
  }

  @Override
  public void teleopPeriodic() {

    /*
    // - - - Select the method of driving - - -
    m_Drivetrain.driveTank(m_Controller.getXboxLeftY(), m_Controller.getXboxRightY());
    m_Drivetrain.driveTank(m_Controller.getLeftStickY(), m_Controller.getRightStickY());
    m_Drivetrain.driveArcade(m_Controller.getXboxLeftY(), m_Controller.getXboxRightX());
    m_Drivetrain.driveArcade(m_Controller.getLeftStickY(), m_Controller.getRightStickX());
    */

    /*
    #################### DIRECTIONS ####################

    To control, set the Drive Mode and Controller Type in Shuffleboard.

    To scale the input of the controller, adjust the value of Constants.kScalingConstantIndex. This number is the index of the
    desired scaling constant from the Constants.kScalingConstants list. A scaling constant of 1 makes the controller give
    linear input from 0 to 1. A scaling constant between 0 and 1 makes the joystick curve concave up, (lower response at higher
    inputs) and a scaling constant above 1 makes the joystick curve concave down (higher response at lower inputs). (When using
    a scaling constant, 0 input always equals 0 output and full input always equals full output. The curve in between is what
    changes.)

    To use turbo mode, hold the trigger on the left joystick when in joystick mode. With the Xbox controller, hold the left
    bumper to use turbo mode.

    A useful Shuffleboard layout for monitoring a few inputs from Togo is saved in the Togo_2022 folder.

    Each Controller Choice on Shuffleboard comes with a deadband, scaling constant, scale when turbo is off, and ramp rate that
    works best for the specified controller. To add new presets, follow the format and instructions for the controllerChoices
    2-D array in the Constants class.
    */

    String newDriveMode = driveModeChooser.getSelected();
    if (!newDriveMode.equals(m_Drivetrain.getDriveMode())) {
      m_Drivetrain.setDriveMode(newDriveMode);
    }

    double newControlChoice = controlChoiceChooser.getSelected();
    if (!(newControlChoice == m_Controller.getControlChoice())) {
      m_Controller.setControlChoice(newControlChoice);
    }

    double leftY = 0;
    double rightY = 0;
    double rightX = 0;

    if (m_Controller.getControllerType() == 0) {
      leftY = -m_Controller.getLeftStickY();
      rightY = -m_Controller.getRightStickY();
      rightX = m_Controller.getRightStickX();
    } else if (m_Controller.getControllerType() == 1) {
      leftY = -m_Controller.getXboxLeftY();
      rightY = -m_Controller.getXboxRightY();
      rightX = m_Controller.getXboxRightX();
    }

    if (m_Drivetrain.getDriveMode().equals(Constants.d_t)) {
      m_Drivetrain.driveTank(leftY, rightY);
    } else if (m_Drivetrain.getDriveMode().equals(Constants.d_a)) {
      m_Drivetrain.driveArcade(leftY, rightX);
    }

    SmartDashboard.putNumber("Controller Type (#)", m_Controller.getControllerType());
    SmartDashboard.putNumber("Deadband", m_Controller.getDeadband());
    SmartDashboard.putNumber("Scale Value", m_Controller.getScalingConstant());
    SmartDashboard.putNumber("Non-Turbo Boundary", m_Drivetrain.getMaxOutputTurboOff());
    if (m_Controller.getControllerType() == 0) {
      SmartDashboard.putBoolean("Turbo Mode", m_Controller.getLeftStickTrigger());
    } else if (m_Controller.getControllerType() == 1) {
      SmartDashboard.putBoolean("Turbo Mode", m_Controller.getXboxLeftBumper());
    }
  }
}
