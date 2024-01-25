package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 14);
    private static Pneumatics instance = new Pneumatics();
    private boolean pistonSetting = false;

    public void Extend() 
    {
        pistonSetting = true;
        piston.set(Value.kForward);
    }

    public void Retract() 
    {
        pistonSetting = false;
        piston.set(Value.kReverse);
    }

    public boolean getPistonSetting()
    {
        return pistonSetting;
    }

    public static Pneumatics getInstance() {
        return instance;
    }
}
