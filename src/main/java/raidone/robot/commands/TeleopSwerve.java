package raidone.robot.commands;

import raidone.robot.Constants;
import raidone.robot.Constants.SwerveConstants;
import raidone.robot.Constants.TeleopConstants;
import raidone.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), TeleopConstants.DRIVE_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), TeleopConstants.DRIVE_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), TeleopConstants.DRIVE_DEADBAND);

        /* Drive */
        swerve.drive(
            translationVal,
            strafeVal, 
            rotationVal * SwerveConstants.MAX_ANGULAR_VEL_MPS, 
            !robotCentricSup.getAsBoolean()
        );
    }
}