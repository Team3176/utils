package frc.robot.commands;

import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Retract extends InstantCommand {
  private Pneumatics m_Pneumatics = Pneumatics.getInstance();
  
  public Retract() {
    addRequirements(m_Pneumatics);
  }

  @Override
  public void initialize() {
    // System.out.println("BallTransferPivotAndRoll.initialize executed.##########################");
    m_Pneumatics.Retract();
  }
}
