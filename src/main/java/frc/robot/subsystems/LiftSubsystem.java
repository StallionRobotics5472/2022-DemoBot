// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */
  
 
  public CANSparkMax rightLift;
  public CANSparkMax leftLift;
   public double currentPosition;
   private double kP = 0.0029;
	private double kI = 0;
	private double kD = 0;
   public PIDController pid = new PIDController(kP, kI, kD);
  public LiftSubsystem() {

  
  rightLift = new CANSparkMax(Constants.rightclimb,MotorType.kBrushless);
  leftLift = new CANSparkMax(Constants.leftClimb,MotorType.kBrushless);
  rightLift.setIdleMode(IdleMode.kBrake);
  leftLift.setIdleMode(IdleMode.kBrake);



  }

  public void climb(double position){
    currentPosition = rightLift.getEncoder().getPosition();
    rightLift.set(pid.calculate(currentPosition, position));
    leftLift.set(pid.calculate(currentPosition, -position));
  }
  public void climb1 (double speed){
    rightLift.set(speed);
    leftLift.set(-speed);
  }
  public double getLiftPosition(){
    return rightLift.getEncoder().getPosition();
  }




  




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
