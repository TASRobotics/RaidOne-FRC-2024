package raidone.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import raidone.robot.subsystems.Swerve;

public class OrdinalTurn extends PIDCommand {

    public OrdinalTurn(double angle, Swerve swerve) {
        super(
                new PIDController(0.08, 0, 0),
                swerve::getHeadingOrdinalTurn,
                angle,
                output -> swerve.drive(new Translation2d(0, 0), output, false, true),
                swerve);

        getController().enableContinuousInput(-180, 180);

        getController().setTolerance(2, 2);

    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
