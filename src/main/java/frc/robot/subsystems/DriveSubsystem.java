// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax leftFrontMotor = new CANSparkMax(DriveConstants.kLeftFrontPort, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DriveConstants.kRightFrontPort, MotorType.kBrushless);
  private final CANSparkMax leftRearMotor = new CANSparkMax(DriveConstants.kLeftRearPort, MotorType.kBrushless);
  private final CANSparkMax rightRearMotor = new CANSparkMax(DriveConstants.kRightRearPort, MotorType.kBrushless);

  private final RelativeEncoder leftFrontEncoder;
  private final RelativeEncoder rightFrontEncoder;
  private final RelativeEncoder leftRearEncoder;
  private final RelativeEncoder rightRearEncoder;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv);
  

  public DriveSubsystem() {

    leftFrontMotor.restoreFactoryDefaults();
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftFrontMotor.setInverted(DriveConstants.kLeftInvert);
    leftFrontEncoder = leftFrontMotor.getEncoder();
    leftFrontEncoder.setPositionConversionFactor(DriveConstants.kPositionConversionFactor);
    leftFrontEncoder.setVelocityConversionFactor(DriveConstants.kVelocityConversionFactor);
    

    rightFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setInverted(DriveConstants.kRightInvert);
    rightFrontEncoder = rightFrontMotor.getEncoder();
    rightFrontEncoder.setPositionConversionFactor(DriveConstants.kPositionConversionFactor);
    rightFrontEncoder.setVelocityConversionFactor(DriveConstants.kVelocityConversionFactor);
    

    leftRearMotor.restoreFactoryDefaults();
    leftRearMotor.setIdleMode(IdleMode.kBrake);
    leftRearMotor.setInverted(DriveConstants.kLeftInvert);
    leftRearEncoder = leftRearMotor.getEncoder();
    leftRearEncoder.setPositionConversionFactor(DriveConstants.kPositionConversionFactor);
    leftRearEncoder.setVelocityConversionFactor(DriveConstants.kVelocityConversionFactor);
    

    rightRearMotor.restoreFactoryDefaults();
    rightRearMotor.setIdleMode(IdleMode.kBrake);
    rightRearMotor.setInverted(DriveConstants.kRightInvert);
    rightRearEncoder = rightRearMotor.getEncoder();
    rightRearEncoder.setPositionConversionFactor(DriveConstants.kPositionConversionFactor);
    rightRearEncoder.setVelocityConversionFactor(DriveConstants.kVelocityConversionFactor);
    
      
  }


  public void drive(double xSpeed, double ySpeed, double rotation) {
    double theta = Math.atan2(ySpeed, xSpeed);
    double power = Math.hypot(xSpeed, ySpeed);

    double sin = Math.sin(theta - Math.PI / 4);
    double cos = Math.cos(theta - Math.PI / 4);
    double max = Math.max(Math.abs(sin), Math.abs(cos));



    double leftFrontPower = power * cos / max + rotation;
    double rightFrontPower = power * sin / max - rotation;
    double leftRearPower = power * sin / max + rotation;
    double rightRearPower = power * cos / max - rotation;

    if ((power + Math.abs(rotation)) > 1) {
      leftFrontPower /= (power + rotation);
      rightFrontPower /= (power + rotation);
      leftRearPower /= (power + rotation);
      rightRearPower /= (power + rotation);
    }

    leftFrontMotor.set(leftFrontPower);
    leftRearMotor.set(leftRearPower);
    rightFrontMotor.set(rightFrontPower);
    rightRearMotor.set(rightRearPower);
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    leftRearEncoder.setPosition(0);
    rightRearEncoder.setPosition(0);
    
  }

  public double getAveragePosition() {
    return (leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition()
      + leftRearEncoder.getPosition() + rightRearEncoder.getPosition()) / 4.0;
  }

  public double getAverageVelocity() {
    return (leftFrontEncoder.getVelocity() + rightFrontEncoder.getVelocity()
      + leftRearEncoder.getVelocity() + rightRearEncoder.getVelocity()) / 4.0;
  }

  public SimpleMotorFeedforward getFeedfoward() {
    return feedforward;
  }

  public void setVoltage(double voltage) {
    leftFrontMotor.setVoltage(voltage);
    rightFrontMotor.setVoltage(voltage);
    leftRearMotor.setVoltage(voltage);
    rightRearMotor.setVoltage(voltage);
  }

  public double getAverageVoltage() {
    return leftFrontMotor.getBusVoltage() * leftFrontMotor.getAppliedOutput();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Average", getAveragePosition());
    SmartDashboard.putNumber("Average Velocity", getAverageVelocity());
    SmartDashboard.putNumber("Average Voltage", getAverageVoltage());
  }
}
