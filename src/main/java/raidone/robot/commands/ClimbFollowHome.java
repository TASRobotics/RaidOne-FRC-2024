package raidone.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import raidone.robot.subsystems.Climb;

public class ClimbFollowHome extends Command {
    private Climb climb;

    public ClimbFollowHome() {
        this.climb = Climb.system();
        // addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.followHome();
    }

    @Override
    public boolean isFinished() {
        // System.out.println(climb.getFollowLimit());
        // return climb.getFollowLimit();
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimbMotor();
    }
}
