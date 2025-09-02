package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonomousHeadingBased extends SequentialCommandGroup {
  public AutonomousHeadingBased(Drivetrain drive) {
    addCommands(
        new DriveDistanceStraight(0.5, 40, drive), new DriveDistanceStraight(-0.5, 40, drive));
  }
}
