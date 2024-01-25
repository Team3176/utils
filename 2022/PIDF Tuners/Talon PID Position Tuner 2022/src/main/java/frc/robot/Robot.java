/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_xboxController = new XboxController(2);
  private int deviceID = 6;
  //private int m_follow_deviceID = 0;    // CAN Id zero disables follow motor mode
  //private boolean m_follow_motor_inverted = true;
  private double m_setPoint;
  private long m_startTime_nanosec = 0;
  private double m_elapsedTime_sec = 0;
  private double overshot = 0;
  private double undershot = 0;
  private TalonSRX m_motor;
  //private TalonSRX m_follow_motor = null;
  //private CANPIDController m_pidController;
  //private CANEncoder m_encoder;
  private boolean m_invert_motor = false;
  private SlewRateLimiter m_rateLimiter;
  private double m_rate_RPMpersecond;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public int kSlotIdx, kPIDLoopIdx, kTimeoutMs;
  public boolean kSensorPhase;
  SendableChooser <String> mode_chooser = new SendableChooser<>();
  private boolean kIsBrakeMode = true;

  public boolean kIsAngler = true;
  public DigitalInput limitSwitchLower;
  public DigitalInput limitSwitchUpper;

  private double beginningPosition;

  @Override
  public void robotInit() {

    // PID coefficients (starting point)
    // Small initial kFF and kP values, probably just big enough to do *something* 
    // and *probably* too small to overdrive an untuned system.
    kFF = 0.0;  
    kP = 0.01;
    kI = 0;
    kD = 0.0;
    kIz = 0;
    kMaxOutput = .05;
    kMinOutput = -.05;
    maxRPM = 5700;
    m_rate_RPMpersecond = 1e10;    // 10 million effectively disables rate limiting
    kSlotIdx = 0;
    kPIDLoopIdx = 0; 
    kTimeoutMs = 30;
    kSensorPhase = true;

    m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_setPoint);

    if (kIsAngler) {
      limitSwitchLower = new DigitalInput(1);
      limitSwitchUpper = new DigitalInput(0);
    }

    //initMotorController(deviceID, m_invert_motor, m_follow_deviceID, m_follow_motor_inverted);
    initMotorController(deviceID, m_invert_motor);

    beginningPosition = m_motor.getSelectedSensorPosition();
    m_setPoint = beginningPosition;

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("CAN Id", deviceID);
    SmartDashboard.putNumber("SetPoint (Encoder Tics)", m_setPoint);
    SmartDashboard.putNumber("Position (Encoder Tics)", beginningPosition - m_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Time to reach Position", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (Position)", 0.0);
    //SmartDashboard.putNumber("Follow CAN Id", m_follow_deviceID);
    //SmartDashboard.putBoolean("Invert Follow Motor", m_follow_motor_inverted);
    SmartDashboard.putBoolean("Invert Lead Motor", m_invert_motor);
    mode_chooser.addOption("Fixed Position Changes (A, B, Y, X buttons)", "fixed");
    mode_chooser.addOption("Variable Position Changes (left stick)", "variable");
    SmartDashboard.putData("Mode", mode_chooser);
    SmartDashboard.putNumber("Applied Output (Motor Voltage)", 0.0);
    SmartDashboard.putNumber("Ramp Rate (RPM/s)", m_rate_RPMpersecond);

    SmartDashboard.putBoolean("Limiter 1", limitSwitchLower.get());
    SmartDashboard.putBoolean("Limiter 2", limitSwitchUpper.get());
  }

  //private void initMotorController(int canId, boolean invert_motor, int follow_canId, boolean follow_inverted) {
  private void initMotorController(int canId, boolean invert_motor) {

    deviceID = canId;
    m_invert_motor = invert_motor;

    // initialize motor
    m_motor = new WPI_TalonSRX(deviceID);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters 
     * in the SPARK MAX to their factory default state. If no argument is passed, these 
     * parameters will not persist between power cycles
     */
    m_motor.configFactoryDefault();
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
    m_motor.setSensorPhase(kSensorPhase);
    if (kIsBrakeMode) {
      m_motor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_motor.setNeutralMode(NeutralMode.Coast);
    }
    m_motor.setInverted(m_invert_motor);

    //if (m_follow_motor != null) {
    //  // If there was a follow motor before, reset it to factory defaults. (disable follow mode)
    //  m_follow_motor.configFactoryDefault();
    //  // make sure motor is in coast mode, in case this motor is mechanically joined to the lead motor
    //  m_follow_motor.setNeutralMode(NeutralMode.Coast);
    //}

    //if (follow_canId != 0) {
    //  // configure follow motor
    //  m_follow_motor = new WPI_TalonSRX(follow_canId);
    //  //m_follow_motor.setNeutralMode(NeutralMode Coast);
    //  m_follow_motor.follow(m_motor, follow_inverted);
    //}
    //else {
    //  m_follow_motor = null;
    //}
    //m_follow_deviceID = follow_canId;
    //m_follow_motor_inverted = follow_inverted;

    /**
     * Setup PID values 
     */

    // Encoder object created to display position values
    //m_encoder = m_motor.getEncoder();

    // Config the peak and nominal outputs
    m_motor.configNominalOutputForward(0, kTimeoutMs);
    m_motor.configNominalOutputReverse(0, kTimeoutMs);
    m_motor.configPeakOutputForward(.05, kTimeoutMs);
    m_motor.configPeakOutputReverse(-.05, kTimeoutMs);

    // set PID coefficients in slot0; typically kFF stays zero
    m_motor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    m_motor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    m_motor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    m_motor.config_IntegralZone(kPIDLoopIdx, kIz, kTimeoutMs);
    m_motor.config_kF(kPIDLoopIdx, kFF, kTimeoutMs);
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    int canId = (int) SmartDashboard.getNumber("CAN Id", 0);
    boolean invert_motor = SmartDashboard.getBoolean("Invert Lead Motor", m_invert_motor);
    //int follow_canId = (int) SmartDashboard.getNumber("Follow CAN Id", 0);
    boolean follow_inverted = (boolean) SmartDashboard.getBoolean("Invert Follow Motor", true);

    //if ((canId != deviceID) || (invert_motor != m_invert_motor) || (follow_canId != m_follow_deviceID)
    //    || (follow_inverted != m_follow_motor_inverted)) {
    if ((canId != deviceID)) {      
      //initMotorController(canId, invert_motor, follow_canId, follow_inverted);
      initMotorController(canId, invert_motor);
      // Reset setpoint to zero if we change anything about the motor configuration. (safety first!)
      m_setPoint = 0;
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_motor.config_kP(kPIDLoopIdx, p, kTimeoutMs); kP = p; }
    if((i != kI)) { m_motor.config_kI(kPIDLoopIdx, i, kTimeoutMs); kI = i; }
    if((d != kD)) { m_motor.config_kD(kPIDLoopIdx, d, kTimeoutMs); kD = d; }
    if((iz != kIz)) { m_motor.config_IntegralZone(kPIDLoopIdx, iz, kTimeoutMs); kIz = iz; }
    if((ff != kFF)) { m_motor.config_kF(kPIDLoopIdx, ff, kTimeoutMs); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_motor.configNominalOutputForward(0, kTimeoutMs);
      m_motor.configNominalOutputReverse(0, kTimeoutMs);
      m_motor.configPeakOutputForward(max, kTimeoutMs);
      m_motor.configPeakOutputReverse(min, kTimeoutMs);
      kMinOutput = min; kMaxOutput = max;
    }
    
    double ramprate = SmartDashboard.getNumber("Ramp Rate (RPM/s)", 0);
    if (ramprate != m_rate_RPMpersecond) {
      m_rateLimiter = new SlewRateLimiter(ramprate, m_setPoint);
      m_rate_RPMpersecond = ramprate;
      SmartDashboard.putNumber("Ramp Rate (RPM/s)", m_rate_RPMpersecond);
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    double setPoint = m_setPoint;
    if (mode_chooser.getSelected() == "variable") {
      // left joystick set RPM set point
      setPoint =  m_xboxController.getLeftY() * maxRPM;
      if (Math.abs(setPoint) < 40) {
        // dead banding. ignore really small joystick inputs
        setPoint = 0;
      }
    }
    else if (mode_chooser.getSelected() == "fixed") {
      // press A, B, Y, X buttons set speed
      // press Right Bumper to stop (set RPM to zero)
      if (m_xboxController.getAButtonPressed()) {
        setPoint = -1024;  // 90 degrees rotation when 1 full rev = 4096 tics
      }
      else if (m_xboxController.getBButtonPressed()) {
        setPoint = -2048; // 180 degrees rotation when 1 full rev = 4096 tics
      }
      else if (m_xboxController.getYButtonPressed()) {
        setPoint = -3072;  // 270 degrees rotation when 1 full rev = 4096 tics
      }
      else if (m_xboxController.getXButtonPressed()) {
        setPoint = -4096;  // 360 degrees rotation when 1 full rev = 4096 tics
      }
      else if (m_xboxController.getRightBumper()) {
        setPoint = beginningPosition;
      } 
    }

    if (m_setPoint != setPoint) {
      // set point changed, start a timer
      m_startTime_nanosec = System.nanoTime();
      m_elapsedTime_sec = 0;
      overshot = 0;
      undershot = 0;

      m_setPoint = setPoint;
    }

    double rawLocation = m_motor.getSelectedSensorPosition();
    double location = rawLocation;

    if (m_elapsedTime_sec == 0) {
      if (Math.abs(location - m_setPoint) < 50) {
          m_elapsedTime_sec = ((double)(System.nanoTime() - m_startTime_nanosec)) / 1000000000.0;
      }
    }

    double error = rawLocation - m_setPoint;

    if (m_elapsedTime_sec > 0) {
      // track max and min error after reaching target location
      if (error > overshot) {
        overshot = error;
      }
      if (error < undershot) {
        undershot = error;
      }
    }

    // Calculate and set new reference RPM
    //double reference_setpoint = m_rateLimiter.calculate(setPoint);
    double reference_setpoint = setPoint;
    if (setPoint == 0) {
       // when we hit  stop, stop immediately. (safety!)
      reference_setpoint = 0;
      m_rateLimiter.reset(0);
    }

    // limit switch checks
    // remember, limit switches are FALSE WHEN PRESSED!!!
    
     
    /* 
    if (kIsAngler)
    {
      if (!limitSwitchLower.get() && error > 0) {
        m_motor.set(ControlMode.PercentOutput, 0.0);
      } else if (!limitSwitchUpper.get() && error < 0) {
        m_motor.set(ControlMode.PercentOutput, 0.0);
      } else {
        m_motor.set(ControlMode.Position, reference_setpoint);
      }
    } else {
      m_motor.set(ControlMode.Position, reference_setpoint);
    }
    */
    m_motor.set(ControlMode.Position, reference_setpoint);

    SmartDashboard.putNumber("SetPoint (Encoder Tics)", reference_setpoint);  // was m_setpoint
    SmartDashboard.putNumber("Position (Encoder Tics)", location);
    SmartDashboard.putNumber("Time to reach Position", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (Position)", error);
    SmartDashboard.putNumber("Applied Output (Motor Voltage)", m_motor.getMotorOutputVoltage());

    SmartDashboard.putBoolean("Limiter Lower", limitSwitchLower.get());
    SmartDashboard.putBoolean("Limiter Upper", limitSwitchUpper.get());

  }

  @Override
  public void robotPeriodic() {
    if (!limitSwitchLower.get()) {
      m_motor.set(ControlMode.PercentOutput, 0.0);
    }
    if (!limitSwitchUpper.get()) {
      m_motor.set(ControlMode.PercentOutput, 0.0);
    }
  }
}