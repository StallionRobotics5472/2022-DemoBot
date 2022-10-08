// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.subsystems.TransportSubsystem;

public class ShootCommand extends CommandBase {
  /** Creates a new PickupCommand. */
private TransportSubsystem transport;
private double kp = 0; 
private double rpm; 
private double angle;
private Limelight limelight;
//private double ticks2RPm = (600.0 / 2048.0);


  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transport = Robot.transport;
    limelight = Robot.limelight;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    if(limelight.getVerticalAngle() > -2.55555555){
      rpm = 4500;
      angle = -2.83;
    }
    else if((limelight.getVerticalAngle()<-2.55555556) & ((limelight.getVerticalAngle() > -10.433333333))) {
      rpm = 5000;
      angle = -2.83;
    }
    else if((limelight.getVerticalAngle()<-10.433333334) & ((limelight.getVerticalAngle() > -15.0555555555))) {
      rpm = 5250;
      angle = -2.65;
    }
    else if((limelight.getVerticalAngle()<-15.0555555556)) {
      rpm = 5100;
      angle = -2.35;
    }
    else{
rpm = 0;
    }
/*if (transport.currentRPM()>(rpm*5/3) & (transport.getHoodPosition()>(angle*0.95))){
 transport.kick(0.5, 0.4);
}
else{
  transport.kick(0, 0);
}*/
    transport.shoot(rpm);
    transport.hood(angle);
    
   
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
