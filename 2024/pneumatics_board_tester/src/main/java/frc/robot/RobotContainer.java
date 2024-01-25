// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pneumatics;
import frc.robot.Controller;
import frc.robot.extendPiston;



public class RobotContainer {
  private final pneumatics m_pneumatics;
  private Controller m_Controller;
  //public Controller operator;

  public RobotContainer() {

    m_Controller = new Controller();
    m_pneumatics = pneumatics.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    
    m_Controller.operator.b().onTrue(new extendPiston());
    
    m_Controller.operator.b().onFalse(new retractPiston());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
