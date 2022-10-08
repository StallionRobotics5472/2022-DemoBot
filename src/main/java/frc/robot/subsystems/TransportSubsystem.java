// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.cert.CertPathValidatorException.BasicReason;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */
  
  public CANSparkMax intake;
  public CANSparkMax rightDropper;
  public CANSparkMax leftDropper;
  public CANSparkMax transport;
  public TalonFX bottomFlywheel;
  public TalonFX topFlywheel;
  public CANSparkMax hood;
  public double hoodPosition;
   public double currentDrop1;
   public double currentDrop2;
   private double kP = 0.025;
   private double kPP = 0.073;
	private double kI = 0;
	private double kD = 0;
  int timeOutMs = 30;
  private static final double HOOD_ANGLE_P = 0.5;
  private static final double HOOD_ANGLE_I = 0;
  private static final double HOOD_ANGLE_D = 5;
  private static final double HOOD_CURRENT_LIMIT = 15.0;

private int loopIDX = 0;

private double f = 0.135;
private double f1 = 0.1;

private double p = 0;

private double i = 0;
public double hoodSetPoint;


private double d = 0;
   public PIDController pid = new PIDController(kP, kI, kD);
   public PIDController pid1 = new PIDController(kPP, kI, kD);
   public DigitalInput ballSensor = new DigitalInput(Constants.Ball_Sensor);
  public TransportSubsystem() {

  intake = new CANSparkMax(Constants.intake, MotorType.kBrushless);
  rightDropper = new CANSparkMax(Constants.rightDropper, MotorType.kBrushless);
  leftDropper = new CANSparkMax(Constants.leftDropper, MotorType.kBrushless);
  transport = new CANSparkMax(Constants.transport, MotorType.kBrushless);
  bottomFlywheel = new TalonFX(Constants.bottomFlywheel);
  topFlywheel = new TalonFX(Constants.topFlywheel);
  hood = new CANSparkMax(Constants.hood, MotorType.kBrushless);
  intake.setIdleMode(IdleMode.kCoast);
  transport.setIdleMode(IdleMode.kBrake);
  hood.setIdleMode(IdleMode.kBrake);
  bottomFlywheel.configFactoryDefault();

  bottomFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeOutMs);
 bottomFlywheel.configNominalOutputReverse(0, timeOutMs);

bottomFlywheel.configPeakOutputForward(1, timeOutMs);

bottomFlywheel.configPeakOutputReverse(-1, timeOutMs);

bottomFlywheel.setInverted(false);

bottomFlywheel.setSensorPhase(false);

bottomFlywheel.config_kF(loopIDX, f, timeOutMs);

bottomFlywheel.config_kP(loopIDX, p, timeOutMs);

bottomFlywheel.config_kI(loopIDX, i, timeOutMs);




  }

  public void  shoot(double speed){
    bottomFlywheel.set(ControlMode.Velocity, -speed);
    topFlywheel.set(ControlMode.Velocity, -speed);
  }
  public double  currentRPM(){
    double motorRPM = bottomFlywheel.getSelectedSensorVelocity();

    double gearRatio = (24/36);
    
    return -(bottomFlywheel.getSelectedSensorVelocity());
  }

  public void pickup(double spin, double move,double dropPosition1, double dropPosition2){
    intake.set(spin);
    transport.set(move);
    currentDrop1 = rightDropper.getEncoder().getPosition();
    currentDrop2 = leftDropper.getEncoder().getPosition();
    rightDropper.set(pid.calculate(currentDrop1, -dropPosition1));
    leftDropper.set(pid.calculate(currentDrop2, dropPosition2));
  }
  public void kick(double spin, double move){
    intake.set(spin);
    transport.set(move);
  }

  public void hood(double hoodSetPoint){
    hoodPosition = hood.getEncoder().getPosition();
    hood.set(pid1.calculate(hoodPosition,hoodSetPoint));
  }


public double getHoodPosition(){
  return hood.getEncoder().getPosition();
}
  public boolean getBallSensor(){
    return ballSensor.get();
  }
  public double dropPostion1() {
    return rightDropper.getEncoder().getPosition();
  }
  public double dropPostion2() {
    return leftDropper.getEncoder().getPosition();
  }

  




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  }

