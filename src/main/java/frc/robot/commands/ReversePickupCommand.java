// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.TransportSubsystem;

public class ReversePickupCommand extends CommandBase {
  /** Creates a new PickupCommand. */
private TransportSubsystem transport;
private double kp = 0; 

  public ReversePickupCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transport = Robot.transport;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transport.pickup(-0.5, -0.5, 0.2,0.2);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
