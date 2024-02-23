package raidone.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import raidone.robot.Constants;
import raidone.robot.subsystems.Arm;

public class TrapezoidGenerator extends TrapezoidProfileCommand {
    // might need to change to m/s???
    private static final Constraints armConstraints = 
        new Constraints(Constants.Arm.MAX_VEL_RPS, Constants.Arm.MAX_ACCEL_RPSS);

    private static final Constraints wristConstraints = 
        new Constraints(Constants.Arm.MAX_VEL_RPS, Constants.Arm.MAX_ACCEL_RPSS);

    public TrapezoidGenerator(Arm arm, State goal) {
        super(
            new TrapezoidProfile(armConstraints), 
            arm::trapezoidToPID, 
            () -> goal, 
            arm::currentState, 
            arm);
    }

    // public TrapezoidGenerator(Wrist wrist, State)
}
