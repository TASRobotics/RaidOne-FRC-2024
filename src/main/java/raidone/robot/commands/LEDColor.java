package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Lighting;

public class LEDColor extends Command{
    private static Lighting LightControl = Lighting.system();
    int[] rgb;

    public LEDColor(int[] rgb){
        this.rgb = rgb;
        addRequirements(LightControl);
    }

    @Override
    public void execute() {
        LightControl.setColor(rgb);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
