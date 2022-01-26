// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  NetworkTable limelightTable;
  double ty, tv, tx, angle, distance;

  /** Creates a new Limelight. */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    ty = limelightTable.getEntry("ty").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);
    tx = limelightTable.getEntry("tv").getDouble(0);
  }

  public double getAngle() {
    if (tx != 0) {
      return tx;
    }
    return 0;
  }

  public double getDistance() {
    if (tv != 0) {
      angle = (LimelightConstants.ANGLE_CAMERA + ty) * Math.PI / 180;
      distance = (LimelightConstants.HEIGHT_TARGET - LimelightConstants.HEIGHT_CAMERA) / Math.tan(angle);
      return distance;
    }
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
