package frc.robot.commands;

import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Extend extends InstantCommand {
  private Pneumatics m_Pneumatics = Pneumatics.getInstance();
  
  public Extend(Pneumatics m_Pneumatics2) {
    addRequirements(m_Pneumatics);
  }

  @Override
  public void initialize() {
    // System.out.println("BallTransferPivotAndRoll.initialize executed.##########################");
    m_Pneumatics.Extend();
  }
}
