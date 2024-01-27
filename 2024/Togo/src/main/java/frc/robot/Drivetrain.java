// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.filter.SlewRateLimiter;

/** Add your docs here. */
public class Drivetrain {

    // private static Drivetrain m_Drivetrain = new Drivetrain();
    private Controller m_Controller;
    private CANSparkMax leftFrontMotor;
    private CANSparkMax leftBackMotor;
    private MotorControllerGroup leftMotorGroup;
    private CANSparkMax rightFrontMotor;
    private CANSparkMax rightBackMotor;
    private MotorControllerGroup rightMotorGroup;
    private DifferentialDrive robotDrive;

    private String driveMode;

    private double maxPercentTurboOff;
    private SlewRateLimiter rateLimiter1;
    private SlewRateLimiter rateLimiter2;

    public Drivetrain()
    {
        leftFrontMotor = new CANSparkMax(Constants.NEO_LEFT_FRONT_CANID, MotorType.kBrushless);
        leftBackMotor = new CANSparkMax(Constants.NEO_LEFT_BACK_CANID, MotorType.kBrushless);
        leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        rightFrontMotor = new CANSparkMax(Constants.NEO_RIGHT_FRONT_CANID, MotorType.kBrushless);
        rightBackMotor = new CANSparkMax(Constants.NEO_RIGHT_BACK_CANID, MotorType.kBrushless);
        rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

        leftMotorGroup.setInverted(true);

        this.driveMode = Constants.d_a;

        this.maxPercentTurboOff = 0;
        this.rateLimiter1 = new SlewRateLimiter(8.0);
        this.rateLimiter2 = new SlewRateLimiter(8.0);
    }

    public String getDriveMode() {
      return this.driveMode;
    }

    public void setDriveMode(String newDriveMode) {
      // stop motors on drive mode change
      this.driveTank(0, 0);
      this.driveMode = newDriveMode;
    }

    public double getMaxOutputTurboOff() {
      return this.maxPercentTurboOff;
    }

    public void setMaxOutputTurboOff(double newOutput) {
      this.maxPercentTurboOff = newOutput;
    }
    
    public void setRamp(double newRamp) {
      this.rateLimiter1 = new SlewRateLimiter(newRamp);
      this.rateLimiter2 = new SlewRateLimiter(newRamp);
      System.out.println("New ramp: " + newRamp + " ############################");
    }

    /**
   * Scales the input from the controller/joystick based on the equation being used and the scaling constant selected
   * @author Jared Brown
   * @param input
   * @return output
   */
  public double scaleInput(double input)
  {
    // "a" is the scaling constant

    // - - - - -
    // Equation 1: f(x) = b(x) + a(x^3)
    // Recommended: 0 <= a <= 1
    // double output = ((1 - Constants.kScalingConstant) * input) + (Constants.kScalingConstant * Math.pow(input, 3));

    // - - - - -
    // Equation 2: f(x) = x^(1/c) ... because this function does weird stuff with negative input, it will run negative input like a
    // positive input and then flip it at the end
    // Recommended: 0.25 <= a <= 5 ... Valid: a > 0
    if (input >= 0) {
      double output = Math.pow(input, 1 / m_Controller.getScalingConstant());
      return output;
    }
    // if negative
    double output = -Math.pow(input * -1, 1 / m_Controller.getScalingConstant());
    return output;
  }

  private double scaleForTurbo(double power)
  {
    if (((m_Controller.getControllerType() == 0) && m_Controller.getLeftStickTrigger()) || 
        ((m_Controller.getControllerType() == 1) && m_Controller.getXboxLeftBumper())) {
      return power;
    }
    return power * this.maxPercentTurboOff;
  }

  /**
   * Drives with robot with tank drive.
   * @param leftY Power on left stick
   * @param rightY Power on right stick
   * @author Jared Brown
   */
  public void driveTank(double leftY, double rightY)
  {
    leftY = this.scaleInput(leftY);
    rightY = this.scaleInput(rightY);
    leftY = this.scaleForTurbo(leftY);
    rightY = this.scaleForTurbo(rightY);

    if (Math.abs(leftY) < m_Controller.getDeadband()) { leftY = 0.0;}
    if (Math.abs(rightY) < m_Controller.getDeadband()) { rightY = 0.0;}

    SmartDashboard.putNumber("leftStickScaled", leftY);
    SmartDashboard.putNumber("rightStickScaled", rightY);

    robotDrive.tankDrive(this.rateLimiter1.calculate(leftY), this.rateLimiter2.calculate(rightY), false);
  }

  /**
   * Drives with robot with arcade drive.
   * @param powerY Power on left stick
   * @param steerX Steering on right stick
   * @author Jared Brown
   */
  public void driveArcade(double powerY, double steerX)
  {
    powerY = this.scaleInput(powerY);
    steerX = this.scaleInput(steerX);
    powerY = this.scaleForTurbo(powerY);

    if (Math.abs(powerY) < m_Controller.getDeadband()) { powerY = 0.0;}
    if (Math.abs(steerX) < m_Controller.getDeadband()) { steerX = 0.0;}
    if (powerY < 0) { steerX *= -1; }

    SmartDashboard.putNumber("leftStickScaled", powerY);
    SmartDashboard.putNumber("rightStickScaled", steerX);

    robotDrive.arcadeDrive(this.rateLimiter1.calculate(powerY), this.rateLimiter2.calculate(steerX), false);
  }

  /*
  public static Drivetrain getInstance() {
    return m_Drivetrain;
  }
  */

  public void setControllerReference(Controller controller) {
    this.m_Controller = controller;
  }

}
