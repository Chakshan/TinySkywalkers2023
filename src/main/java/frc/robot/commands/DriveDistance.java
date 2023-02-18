// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistance extends ProfiledPIDCommand {
  /** Creates a new DriveDistance. */
  public DriveDistance(DriveSubsystem drive, double distance) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.3, 0.3)),
        // This should return the measurement
        drive::getAveragePosition,
        // This should return the goal (can also be a constant)
        distance,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          double feedfoward = drive.getFeedfoward().calculate(setpoint.velocity);
          double voltage = MathUtil.clamp((feedfoward + output), -6.0, 6.0);
          drive.setVoltage(voltage);

        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drive);
    drive.resetEncoders();

    getController().setTolerance(0.05);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
