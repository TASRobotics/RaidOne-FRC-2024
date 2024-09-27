// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidone.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidone.lib.util.ColorConverter;
import raidone.robot.Constants;
import raidone.robot.RobotContainer;
import raidone.robot.RobotContainer.RobotState;
import raidone.robot.subsystems.Intake.IntakeStateEnum;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class Lights extends SubsystemBase {
    private static Lights lights = new Lights();
    private final CANdle m_candle = new CANdle(Constants.Lights.CANdleID, "rio"); 
    private final int NUM_LEDS= 128;
    private int hue = 0; //hue is used to set all colors to a particular hue, google CHSV for colors->numbers
    private Animation m_toAnimate = null;  //the animation that is currently playing
    private boolean m_clearAllAnims = false;
    //private int m_candleChannel = 0;
    //private boolean m_last5V = false;
    //private AnimationTypes m_currentAnimation;
    private boolean m_setAnim = false; //used to prevent unneccesary refreshes in periodic()
    private Intake intake = Intake.system(); 
    RobotContainer.RobotState robotState = RobotContainer.getRobotState();


    public enum AnimationTypes { //list of all possible animations by name
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAllRed,
        SetAllBlue
    }
    
    

    private Lights() {
        changeAnimation(AnimationTypes.Twinkle); //set default animation to Twinkle
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        //m_candle.configV5Enabled(m_last5V);
        m_candle.configAllSettings(configAll, 100);
        //robotState = RobotState.IDLE;
        System.out.println("Lights init");

        
    }


    // public void decrementAnimation() {
    //     switch(m_currentAnimation) {
    //         case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
    //         case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
    //         case Larson: changeAnimation(AnimationTypes.Fire); break;
    //         case Rainbow: changeAnimation(AnimationTypes.Larson); break;
    //         case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
    //         case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
    //         case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
    //         case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
    //         case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
    //         case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    //     }
    // }
    public void setColors() {


        //changeAnimation()
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        //m_currentAnimation = toChange; //this is only used when working with multiple animations on the same strip
        
        switch(toChange) //this is where the enum is converted into the actual animation type, if adding new animation, configure here
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, NUM_LEDS, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, NUM_LEDS, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, NUM_LEDS, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, NUM_LEDS);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, NUM_LEDS);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, NUM_LEDS);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, NUM_LEDS);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, NUM_LEDS, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, NUM_LEDS, TwinkleOffPercent.Percent100);
                break;
            case SetAllRed:
                hue = 0;
                m_toAnimate = null; //set to null because no animation is playing
                break;
            case SetAllBlue:
                hue = 160;
                m_toAnimate = null;
                break;
        }
        SmartDashboard.putString("Animation", toChange.toString());
        //System.out.println("Changed to " + m_currentAnimation.toString());
    }

    @Override
    public void periodic() {
        IntakeStateEnum intakeState = intake.getState();
        
        if(intakeState == IntakeStateEnum.IDLE_NO_NOTE || intakeState == IntakeStateEnum.RUNNING_NO_NOTE){
            changeAnimation(AnimationTypes.ColorFlow);
            //RobotContainer.setRobotState( RobotContainer.RobotState.HOMED_NO_NOTE);
            RobotContainer.setRobotState(RobotContainer.RobotState.HOMED_NO_NOTE);
        } else if (intakeState == IntakeStateEnum.IDLE_HAS_NOTE || intakeState == IntakeStateEnum.RUNNING_HAS_NOTE){
            changeAnimation(AnimationTypes.Fire);
            RobotContainer.setRobotState(RobotContainer.RobotState.HOMED_HAS_NOTE);
            //RobotContainer.setRobotState(RobotContainer.RobotState.HOMED_HAS_NOTE);
        }
        


        if(m_toAnimate == null) {
            if(!m_setAnim) {   /* Only setLEDs once, because every set will transmit a frame */
                int[] rgb = ColorConverter.hueToRGB(hue); //convert hue to RGB
                m_candle.setLEDs(rgb[0], rgb[1], rgb[2]); 
                m_setAnim = true;
            }
        } else {
            m_candle.animate(m_toAnimate);
            m_setAnim = false;
        }
        
        if(m_clearAllAnims) {
            m_clearAllAnims = false;
            for(int i = 0; i < 10; ++i) {
                m_candle.clearAnimation(i);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
      
        // This method will be called once per scheduler run during simulation
    }

    public static Lights system() {
        return lights;
    }

    
}
