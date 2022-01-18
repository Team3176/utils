/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_xboxController = new XboxController(0);
  // private PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  private PowerDistribution m_pdp = new PowerDistribution(0, ModuleType.kCTRE);
  private int deviceID = 1;
  //private int m_follow_deviceID = 0;    // CAN Id zero disables follow motor mode
  //private boolean m_follow_motor_inverted = true;
  private double m_setPoint = 0;
  private long m_startTime_nanosec = 0;
  private double m_elapsedTime_sec = 0;
  private double overshot = 0;
  private double undershot = 0;
  private TalonFX m_motor;
  //private CANSparkMax m_follow_motor = null;
  //private CANPIDController m_pidController;
  //private CANEncoder m_encoder;
  private boolean m_invert_motor = true;
  private SlewRateLimiter m_rateLimiter;
  private double m_rate_RPMpersecond;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public int kSlotIdx, kPIDLoopIdx, kTimeoutMs, kSensorUnitsPerRevolution;
  public 
  SendableChooser <String> mode_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    // PID coefficients (starting point)
    // Small initial kFF and kP values, probably just big enough to do *something* 
    // and *probably* too small to overdrive an untuned system.
    // PID Gains may have to be adjusted based on the responsiveness of control loop.
    // kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
    kFF = 1023.0/20660.0;  
    kP = 0.1;
    kI = 0.001;
    kD = 5.0;
    kIz = 300;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    maxRPM = 5700;
    kTimeoutMs = 30;
    kSensorUnitsPerRevolution = 2048;
    m_rate_RPMpersecond = 1e10;    // 10 million effectively disables rate limiting

    m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_setPoint);

    //initMotorController(deviceID, m_invert_motor, m_follow_deviceID, m_follow_motor_inverted);
    initMotorController(deviceID, m_invert_motor);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("CAN Id", deviceID);
    SmartDashboard.putNumber("SetPoint (RPM)", m_setPoint);
    SmartDashboard.putNumber("Velocity (RPM)", m_motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Total Current (Amp)", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total Power (W)", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Time to reach SetPoint", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (RPM)", 0.0);
    //SmartDashboard.putNumber("Follow CAN Id", m_follow_deviceID);
    //SmartDashboard.putBoolean("Invert Follow Motor", m_follow_motor_inverted);
    SmartDashboard.putBoolean("Invert Lead Motor", m_invert_motor);
    mode_chooser.addOption("Fixed RPM (A, B, Y, X buttons)", "fixed");
    mode_chooser.addOption("Variable RPM (left stick)", "variable");
    SmartDashboard.putData("Mode", mode_chooser);
    SmartDashboard.putNumber("Applied Output (Motor Voltage)", 0.0);
    SmartDashboard.putNumber("Ramp Rate (RPM/s)", m_rate_RPMpersecond);

  }

  //private void initMotorController(int canId, boolean invert_motor, int follow_canId, boolean follow_inverted) {
  private void initMotorController(int canId, boolean invert_motor) {

    deviceID = canId;
    m_invert_motor = invert_motor;

    // initialize motor
    m_motor = new TalonFX(deviceID);


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters 
     * in the SPARK MAX to their factory default state. If no argument is passed, these 
     * parameters will not persist between power cycles
     */
    
    // Factory default all HW to prevent unexpected behaviour
    m_motor.configFactoryDefault();
    
    // Config neutral deadband to be smallest possible
    m_motor.configNeutralDeadband(0.001);

    // Config sensor used for Primary PID (Velocity)
    m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    // m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
    m_motor.setInverted(m_invert_motor);

    //if (m_follow_motor != null) {
    //  // If there was a follow motor before, reset it to factory defaults. (disable follow mode)
    //  m_follow_motor.restoreFactoryDefaults();
    //  // make sure motor is in coast mode, in case this motor is mechanically joined to the lead motor
    //  m_follow_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //}

    //if (follow_canId != 0) {
    //  // configure follow motor
    //  m_follow_motor = new CANSparkMax(follow_canId, MotorType.kBrushless);
    //  m_follow_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //  m_follow_motor.follow(m_motor, follow_inverted);
    //}
    //else {
    //  m_follow_motor = null;
    //}
    //m_follow_deviceID = follow_canId;
    //m_follow_motor_inverted = follow_inverted;

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    //m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    //m_encoder = m_motor.getEncoder();

    // Config peak and nominal outputs
    m_motor.configNominalOutputForward(0, kTimeoutMs);
    m_motor.configNominalOutputReverse(0, kTimeoutMs);
    m_motor.configPeakOutputForward(kMaxOutput, kTimeoutMs);
    m_motor.configPeakOutputReverse(kMinOutput, kTimeoutMs); 

    // set PID coefficients
    m_motor.config_kF(kPIDLoopIdx, kFF, kTimeoutMs);
    m_motor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    m_motor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    m_motor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    //m_motor.config_IntegralZone(kPIDLoopIdx, kIz, kTimeoutMs);
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
    //boolean follow_inverted = (boolean) SmartDashboard.getBoolean("Invert Follow Motor", true);

    //if ((canId != deviceID) || (invert_motor != m_invert_motor) || (follow_canId != m_follow_deviceID)
    //    || (follow_inverted != m_follow_motor_inverted)) {
    if ((canId != deviceID) || (invert_motor != m_invert_motor)) {
      initMotorController(canId, invert_motor);

      // Reset RPM to zero if we change anything about the motor configuration. (safety first!)
      m_setPoint = 0;
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((ff != kFF)) { m_motor.config_kF(kPIDLoopIdx, ff, kTimeoutMs); kFF = ff; }
    if((p != kP)) { m_motor.config_kP(kPIDLoopIdx, p, kTimeoutMs); kP = p; }
    if((i != kI)) { m_motor.config_kI(kPIDLoopIdx, i, kTimeoutMs); kI = i; }
    if((d != kD)) { m_motor.config_kD(kPIDLoopIdx, d, kTimeoutMs); kD = d; }
    //if((iz != kIz)) { m_motor.config_IntegralZone(kPIDLoopIdx, iz, kTimeoutMs); kIz = iz; }
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
    double targetRPM = m_setPoint;
    double setPoint = targetRPM * kSensorUnitsPerRevolution / 600; // Equates to encoder tics per 100 ms for the given targetRPM value
    if (mode_chooser.getSelected() == "variable") {
      // left joystick set RPM target 
      targetRPM = m_xboxController.getLeftY() * maxRPM;
      setPoint = targetRPM * kSensorUnitsPerRevolution / 600; // Equates to encoder tics per 100 ms for the given targetRPM value
      if (Math.abs(setPoint) < 40) {
        // dead banding. ignore really small joystick inputs
        targetRPM = 0;
      }
    }
    else if (mode_chooser.getSelected() == "fixed") {
      // press A, B, Y, X buttons set speed
      // press Right Bumper to stop (set RPM to zero)
      if (m_xboxController.getAButtonPressed()) {
        targetRPM = 1000;
      }
      else if (m_xboxController.getBButtonPressed()) {
        targetRPM = 2000;
      }
      else if (m_xboxController.getYButtonPressed()) {
        targetRPM = 3000;
      }
      else if (m_xboxController.getXButtonPressed()) {
        targetRPM = 4000;
      }
      else if (m_xboxController.getRightBumperPressed()) {
        targetRPM = 0;
      } 
    }
    setPoint = targetRPM * kSensorUnitsPerRevolution / 600; // Equates to encoder tics per 100 ms for the given targetRPM value

    if (m_setPoint != targetRPM) {
      // set point changed, start a timer
      m_startTime_nanosec = System.nanoTime();
      m_elapsedTime_sec = 0;
      overshot = 0;
      undershot = 0;

      m_setPoint = targetRPM;
    }

    double encoderTicsPer100ms = m_motor.getSelectedSensorVelocity();
    double rpm = encoderTicsPer100ms * 600 / kSensorUnitsPerRevolution;

    if (m_elapsedTime_sec == 0) {
      if (Math.abs(rpm - m_setPoint) < 50) {
          m_elapsedTime_sec = ((double)(System.nanoTime() - m_startTime_nanosec)) / 1000000000.0;
      }
    }

    //double error = encoderTicsPer100ms - m_setPoint;
    double error = m_motor.getClosedLoopError(kPIDLoopIdx);
    double rpmError = error * 600 / kSensorUnitsPerRevolution;

    if (m_elapsedTime_sec > 0) {
      // track max and min error after reaching target rpm
      if (rpmError > overshot) {
        overshot = rpmError;
      }
      if (rpmError < undershot) {
        undershot = rpmError;
      }
    }

    // Calculate and set new reference Velocity (units = tics per 100ms)
    double reference_setpoint = m_rateLimiter.calculate(setPoint);
    if (setPoint == 0) {
       // when we hit  stop, stop immediately. (safety!)
      reference_setpoint = 0;
      m_rateLimiter.reset(0);
    }
    m_motor.set(TalonFXControlMode.Velocity, reference_setpoint); 
    double rpm_reference_setpoint = reference_setpoint * 600 / kSensorUnitsPerRevolution;

    SmartDashboard.putNumber("SetPoint (RPM)", rpm_reference_setpoint);  // was m_setpoint
    SmartDashboard.putNumber("Velocity (RPM)", rpm);
    SmartDashboard.putNumber("Total Current (Amp)", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total Power (W)", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Time to reach SetPoint", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (RPM)", rpmError);
    SmartDashboard.putNumber("Applied Output (Motor Voltage)", m_motor.getMotorOutputVoltage());
  }
}
