// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class SetAngle extends CommandBase {

  private SwerveModule swerveModule;
  private SwerveDrive swerve;

  /** Creates a new SetAngle. */
  public SetAngle(SwerveModule swerveModule, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveModule = new SwerveModule(1, 5, DriveConstants.FRONT_LEFT_ENCODER_ID, false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerveModule.setAngle(new SwerveModuleState(0, new Rotation2d(90)));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(swerveModule.getRotatePosition() >= 85 && swerveModule.getRotatePosition() < 90){
      return true;
    }
    return false;
  }
}
