package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public void robotInit() {
        // port set to 9
        m_led =  new AddressableLED(9);
        
        //length of LED strip set to 60
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());


        m_led.setData(m_ledBuffer);
        m_led.start();
        


        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // set Led RGB values for the color orange
            m_ledBuffer.setRGB(i, 255, 65, 0);


        }  
        
        m_led.setData(m_ledBuffer);
    }

}
