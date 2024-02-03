package raidone.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import raidone.robot.commands.DrivePath;
import raidone.robot.subsystems.Swerve;

public class Autos implements Sendable {
    
    private final Swerve swerve;

    private final SendableChooser<Command> chooser;

    public Autos(Swerve swerve) {
        this.swerve = swerve;

        chooser = new SendableChooser<Command>();
        configureAutos();
    }

    private void configureAutos() {
        chooser.setDefaultOption("No auto", null);
        chooser.addOption("Test auto", testAuto());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        chooser.initSendable(builder);
    }

    public Command get() {
        return chooser.getSelected();
    }

    public Command testAuto() {
        return Commands.sequence(
            new DrivePath(this.swerve, PathPlannerPath.fromPathFile("Path 1"))
                .andThen(new WaitCommand(0.5))
                .andThen(swerve.setX())
        );
    }

}
