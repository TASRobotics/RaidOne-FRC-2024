package raidone.robot.commands;

import raidone.robot.Constants.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private raidone.robot.subsystems.Swerve swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private double translationVal;
    private double strafeVal;
    private double rotationVal;

    public TeleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup,DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve = raidone.robot.subsystems.Swerve.system();
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Swerve.STICK_DEADBAND);
        strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Swerve.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Swerve.STICK_DEADBAND);

        /* Drive */
        swerve.drive(
                new Translation2d(translationVal * translationVal * translationVal, strafeVal * strafeVal * strafeVal).times(Swerve.MAX_SPEED),
                rotationVal * rotationVal * rotationVal * Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}