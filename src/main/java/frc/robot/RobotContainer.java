// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCharacterization;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;


public class RobotContainer {

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final DriveSubsystem drive = new DriveSubsystem();

  public RobotContainer() {
    configureBindings();

    drive.setDefaultCommand(
      Commands.run(
        () -> {
          double xSpeed = joystick.getX() * 0.5;
          double ySpeed = -joystick.getY() * 0.5;
          double rotation = joystick.getZ() * 0.5;
          drive.drive(xSpeed, ySpeed, rotation);
        }, drive));


  }

  private void configureBindings() {
    // joystick.button(1).onTrue(new DriveCharacterization(drive));
  }


  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
