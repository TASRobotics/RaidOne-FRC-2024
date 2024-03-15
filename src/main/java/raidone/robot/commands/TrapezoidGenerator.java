package raidone.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import raidone.robot.Constants;
import raidone.robot.subsystems.Arm;
import raidone.robot.subsystems.Wrist;

public final class TrapezoidGenerator {

    public static TrapezoidProfileCommand armProfile(State goal, boolean auto) {
        return new TrapezoidProfileCommand(
                auto ? Constants.Arm.AUTO_ARM_PROFILE : Constants.Arm.ARM_PROFILE,
                Arm.system()::trapezoidToPID,
                () -> goal,
                Arm.system()::currentState,
                Arm.system());
    }

    public static TrapezoidProfileCommand wristProfile(State goal, boolean auto) {
        return new TrapezoidProfileCommand(
                auto ? Constants.Wrist.AUTO_WRIST_PROFILE : Constants.Wrist.WRIST_PROFILE,
                Wrist.system()::trapezoidToPID,
                () -> goal,
                Wrist.system()::currentState,
                Wrist.system());
    }
}
