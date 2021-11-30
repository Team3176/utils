package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {
    private static Controller instance = new Controller();
    private XboxController duke;
    private JoystickButton xButton;

    public Controller() {
        duke = new XboxController(2);
        xButton = new JoystickButton(duke, Button.kX.value);
    }

    public JoystickButton getDuke_XButton() {
        return xButton;
    }

    public static Controller getInstance() {
        return instance;
    }
}
