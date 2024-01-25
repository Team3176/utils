package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics {
    private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
    private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    private static Pneumatics instance = new Pneumatics();

    public void Extend() {
        piston.set(Value.kForward);
    }

    public void Retract() {
        piston.set(Value.kReverse);
    }

    public static Pneumatics getInstance() {
        return instance;
    }
}
