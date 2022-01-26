// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax encoderTop, encoderBottom;
  WPI_VictorSPX motorTop, motorBottom;
  public Shooter() {
    encoderTop = new CANSparkMax(5);
    encoderBottom = new CANSparkMax(6);
    motorTop = new WPI_VictorSPX(7);
    motorBottom = new WPI_VictorSPX(8);

    PIDController pid = new PIDController(constants.kP, constants.kI, constants.kD);


    topShooterMotors = new SpeedControllerGroup(encoderTop, motorTop);
    bottomShooterMotors = new SpeedControllerGroup(encoderBottom, motorBottom);
  }

  public void set(int setpoint) {
    motorTop.set(pid.calculate(encodertop.getrate() * VEL_RPM_CONVERSION, setpoint));
    motorBottom.set(pid.calculate(encoderbottom.getrate() * VEL_RPM_CONVERSION, setpoint));
  }

  public void stop()
  {
    topShooterMotors.set(0);
    bottomShooterMotors.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
