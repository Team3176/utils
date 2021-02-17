package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Robot extends TimedRobot {
  Compressor c = new Compressor(); //Creates a Compressor Object
  Joystick j = new Joystick(0); //Creates a Joystick Object. The parameter is the id of the joystick in the driver station

  /*
   * Note: the first number is the opening side and the second number is the closing side (Values are from the PCM)
   */

  DoubleSolenoid ds1 = new DoubleSolenoid(0,7);
  DoubleSolenoid ds2 = new DoubleSolenoid(1,6);
  DoubleSolenoid ds3 = new DoubleSolenoid(2,3);
  DoubleSolenoid ds4 = new DoubleSolenoid(4,5);


  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {
    c.start(); //Starts the compressor at Init
  }

  @Override
  public void teleopPeriodic() {
    if(j.getRawButton(1)) {c.start();} //Trigger starts the Compressor
    if(j.getRawButton(2)) {c.stop();} //Side Button stops the Compressor

    /*
     * Look for the labeled buttons with the numbers below
     */

    if(j.getRawButton(3)) {ds1.set(Value.kForward);}
    if(j.getRawButton(4)) {ds1.set(Value.kReverse);}
    if(j.getRawButton(5)) {ds2.set(Value.kForward);}
    if(j.getRawButton(6)) {ds2.set(Value.kReverse);}
    if(j.getRawButton(7)) {ds3.set(Value.kForward);}
    if(j.getRawButton(8)) {ds3.set(Value.kReverse);}
    if(j.getRawButton(9)) {ds4.set(Value.kForward);}
    if(j.getRawButton(10)) {ds4.set(Value.kReverse);}
  }
}
