package raidone.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import raidone.robot.subsystems.Swerve;
import raidone.robot.commands.DrivePath;

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
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        chooser.initSendable(builder);
    }

    public Command get() {
        return chooser.getSelected();
    }

    public Command testAuto() {
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test");

        return Commands.sequence(
            new DrivePath(swerve, path1, true)
        );
    }

}
