package raidone.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import raidone.robot.subsystems.Swerve;

public class OrdinalTurn extends PIDCommand {

    public OrdinalTurn(double angle) {
        super(
                new PIDController(0.13, 0.0, 0.01),
                Swerve.system()::getHeadingOrdinalTurn,
                angle,
                output -> Swerve.system().drive(new Translation2d(0, 0), output, false, true),
                Swerve.system());

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(3, 2);

    }

    @Override
    public boolean isFinished() {
        System.out.println(getController().getPositionError());
        return getController().atSetpoint();
    }
}
