/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;


public class RobotContainer {

  private final Chassis chassis = new Chassis();

  public RobotContainer() { configureButtonBindings(); }

  private void configureButtonBindings() { }


  public Command getAutonomousCommand() {

    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, 
    Constants.kaVoltSecondsSquaredPerMeter).setKinematics(chassis.getKinematicks());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(
        new Pose2d(),
        new Pose2d(1,0,new Rotation2d())
        ),
      config
    );

    RamseteCommand command = new RamseteCommand(
      trajectory, 
      chassis::getPose, 
      new RamseteController(Constants.beta,Constants.beta), 
      chassis.getFeedforward(), 
      chassis.getKinematicks(), 
      chassis::getSpeeds, 
      chassis.getLeftPID(), 
      chassis.getRightPID(), 
      chassis::setOutput, 
      chassis
      );

      return command.andThen(() -> chassis.setOutput(0, 0));

  }
}
