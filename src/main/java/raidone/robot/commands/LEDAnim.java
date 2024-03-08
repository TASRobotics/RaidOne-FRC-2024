package raidone.robot.commands;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Lighting;

public class LEDAnim extends Command{
    private static Lighting LightControl = Lighting.system();
    Animation anim;

    public LEDAnim(Animation anim){
        this.anim = anim;
        addRequirements(LightControl);
    }

    @Override
    public void execute() {
        LightControl.setAnim(anim);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
