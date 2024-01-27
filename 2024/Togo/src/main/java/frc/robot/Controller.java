// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Controller {

  // private static Controller m_Controller = new Controller();
  private Drivetrain m_Drivetrain;
  private final XboxController m_driverController;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  private double controlChoice;

  private double controllerType;
  private double deadband;
  private double scalingConstant;

  public Controller()
  {
    // this.m_Drivetrain = Drivetrain.getInstance();
    m_driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
    leftJoystick = new Joystick(Constants.JOYSTICK_LEFT_PORT);
    rightJoystick = new Joystick(Constants.JOYSTICK_RIGHT_PORT);

    this.controlChoice = 0;

    this.controllerType = 0;
    this.deadband = 0.10;
    this.scalingConstant = 1;
  }

  public double getControlChoice() {
    return this.controlChoice;
  }
  
  /**
   * Returns "0" (joysticks) or "1" (xbox controller) to signal which type of controller is being used.
   * @return Type of controller
   */
  public double getControllerType() {
    return this.controllerType;
  }

  public void setControlChoice(double newControlChoice) {
    // stop motors on control choice change
    System.out.println(m_Drivetrain);
    m_Drivetrain.driveTank(0.0, 0.0);
    this.controlChoice = newControlChoice;

    // search through choices of preset values, looking for the set with the reference index matching the chosen option
    for (double[] preset : Constants.controllerChoices) {
      if (preset[0] == newControlChoice) {
        this.controllerType = preset[1];
        this.deadband = preset[2];
        this.scalingConstant = preset[3];
        m_Drivetrain.setMaxOutputTurboOff(preset[4]);
        m_Drivetrain.setRamp(preset[5]);
        break;
      }
    }
  }

  public double getLeftStickY() { return leftJoystick.getY(); }
  public double getRightStickY() { return rightJoystick.getY(); }
  public double getRightStickX() { return rightJoystick.getX(); }

  public double getXboxLeftY() { return m_driverController.getLeftY(); }
  public double getXboxRightY() { return m_driverController.getRightY(); }
  public double getXboxRightX() { return m_driverController.getRightX(); }

  public boolean getLeftStickTrigger() { return leftJoystick.getTrigger(); }
  public boolean getXboxLeftBumper() { return m_driverController.getLeftBumper(); }

  public double getDeadband() { return this.deadband; }
  public double getScalingConstant() { return this.scalingConstant; }

  // public static Controller getInstance() { return m_Controller; }

  public void setDrivetrainReference(Drivetrain drivetrain) {
    this.m_Drivetrain = drivetrain;
  }
}
