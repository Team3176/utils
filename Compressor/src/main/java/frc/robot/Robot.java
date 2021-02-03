package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import double solenoid
//import double solenoid value

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Compressor c = new Compressor();
  Joystick joy = new Joystick(0);
//create 4 double solenoids and leave notes

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {
    c.start();
  }

  @Override
  public void teleopPeriodic() {
    if(joy.getRawButton(1)) {
      c.stop();
    }
//add a start compressor, add buttons to test each of the 4 double solenoids
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
}
