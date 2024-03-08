package raidone.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase{
    private CANdle ledControl = new CANdle(0);
    private CANdleConfiguration config = new CANdleConfiguration();

    private static Lighting LightControl = new Lighting();

    private Lighting(){
        config.stripType = LEDStripType.RGB;
        ledControl.configAllSettings(config);
    }

    public void setAnim(Animation anim){
        ledControl.animate(anim);
    }

    public void setColor(int rgb[]){
        ledControl.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    public static Lighting system(){
        return LightControl;
    }
}
