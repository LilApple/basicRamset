/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

  // Creating electric components
  private final PWMTalonSRX leftMaster = new PWMTalonSRX(Constants.leftSideSpeedControllers[0]);
  private final PWMTalonSRX leftSlave = new PWMTalonSRX(Constants.leftSideSpeedControllers[1]);
  private final PWMTalonSRX rightMaster = new PWMTalonSRX(Constants.rightSideSpeedControllers[0]);
  private final PWMTalonSRX rightSlave = new PWMTalonSRX(Constants.rightSideSpeedControllers[1]);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0,0,0);

  PIDController leftController = new PIDController(1, 0, 0);
  PIDController rightController = new PIDController(1, 0, 0);

  Pose2d pose;

  public Chassis() {
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getSpeed() / Constants.gearRatio * 2 * Math.PI * Constants.wheelCircumfrence / 60,
      rightMaster.getSpeed() / Constants.gearRatio * 2 * Math.PI * Constants.wheelCircumfrence / 60
    );
  }

  public SimpleMotorFeedforward getFeedforward(){ return feedforward; }

  public PIDController getLeftPID(){ return leftController; }

  public PIDController getRightPID(){ return rightController; }
  
  public DifferentialDriveKinematics getKinematicks(){ return kinematics; }
  
  public Rotation2d getHeading(){ return Rotation2d.fromDegrees(-gyro.getAngle()); }

  public Pose2d getPose(){ return pose; }

  public void setOutput(double leftPower, double rightPower) {
    leftMaster.set(leftPower);
    rightMaster.set(rightPower);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
  }
}
