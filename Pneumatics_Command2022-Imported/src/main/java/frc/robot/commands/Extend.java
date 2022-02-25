package frc.robot.commands;

import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Extend extends InstantCommand {
  private Pneumatics m_Pneumatics = Pneumatics.getInstance();
  
  public Extend() {
    addRequirements(m_Pneumatics);
  }

  @Override
  public void initialize() {
      System.out.println("Extend pistonSetting: " + m_Pneumatics.getPistonSetting());
      m_Pneumatics.Extend();
      System.out.println("Extend pistonSetting: " + m_Pneumatics.getPistonSetting());
    
  }
}
