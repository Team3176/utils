// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kickerbox extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonSRX motorOne;
  TalonSRX motorTwo;
  TalonSRX motorThree;
  double motorOnePercent;
  double motorTwoPercent;
  double motorThreePercent;
  static Kickerbox instance;
  public Kickerbox() 
  {
    motorOne = new TalonSRX(0);
    motorTwo = new TalonSRX(1);
    motorThree = new TalonSRX(2);
    SmartDashboard.putNumber("motorOneOutput%", motorOnePercent);
    SmartDashboard.putNumber("motorTwoOutput%", motorTwoPercent);
    SmartDashboard.putNumber("motorThreeOutput%", motorThreePercent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorOnePercent = SmartDashboard.getNumber("motorOneOutput%", motorOnePercent);
    motorTwoPercent = SmartDashboard.getNumber("motorTwoOutput%", motorTwoPercent);
    motorThreePercent = SmartDashboard.getNumber("motorThreeOutput%", motorThreePercent);
  }

  public void MotorSpin(int motorWanted)
  {
    if (motorWanted == 1)
    {
      motorOne.set(ControlMode.PercentOutput, motorOnePercent);
    }
    else if (motorWanted == 2)
    {
      motorTwo.set(ControlMode.PercentOutput, motorTwoPercent);
    }
    else if (motorWanted == 3)
    {
      motorThree.set(ControlMode.PercentOutput, motorThreePercent);
    }
  }

  public void MotorRevSpin(int motorWanted)
  {
    if (motorWanted == 1)
    {
      motorOne.set(ControlMode.PercentOutput, -motorOnePercent);
    }
    else if (motorWanted == 2)
    {
      motorTwo.set(ControlMode.PercentOutput, -motorTwoPercent);
    }
    else if (motorWanted == 3)
    {
      motorThree.set(ControlMode.PercentOutput, -motorThreePercent);
    }
  }

  public void MotorStop(int motorWanted)
  {
    if (motorWanted == 1)
    {
      motorOne.set(ControlMode.PercentOutput, 0);
    }
    else if (motorWanted == 2)
    {
      motorTwo.set(ControlMode.PercentOutput, 0);
    }
    else if (motorWanted == 3)
    {
      motorThree.set(ControlMode.PercentOutput, 0);
    }
  }

  public static Kickerbox getInstance()
  {
    if (instance == null) {instance = new Kickerbox();}
    return instance;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
