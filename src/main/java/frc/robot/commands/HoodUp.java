// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class HoodUp extends CommandBase {
  /** Creates a new PickupCommand. */
private TransportSubsystem transport;
private LiftSubsystem lift;
private double kp = 0; 
private double ki = 0;
private double kd = 0;
PIDController pid = new PIDController(kp, ki, kd);

  public HoodUp() {
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
   //transport.hood(2000);
  //  transport.hood1(2000);
transport.hood(0);

   
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
