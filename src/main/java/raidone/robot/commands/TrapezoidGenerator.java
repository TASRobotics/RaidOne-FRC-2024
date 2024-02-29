package raidone.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import raidone.robot.Constants;
import raidone.robot.subsystems.Arm;
import raidone.robot.subsystems.Wrist;

public final class TrapezoidGenerator {

    public static TrapezoidProfileCommand armProfile(State goal) {
        return new TrapezoidProfileCommand(
            Constants.Arm.ARM_Profile, 
            Arm.system()::trapezoidToPID, 
            () -> goal, 
            Arm.system()::currentState, 
            Arm.system());
    }

    public static TrapezoidProfileCommand wristProfile(State goal) {
        return new TrapezoidProfileCommand(
            Constants.Wrist.WRIST_Profile,
            Wrist.system()::trapezoidToPID,
            () -> goal,
            Wrist.system()::currentState,
            Wrist.system());
    }
}
