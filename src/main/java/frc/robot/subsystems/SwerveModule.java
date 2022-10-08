// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class    SwerveModule {
    private final CANSparkMax angleMotor;
    private final TalonFX driveMotor; 

    private final RelativeEncoder angleEncoder; 

    private final PIDController turningPidController; 

    // private final DutyCycle encoder; 
    // private final DigitalSource magEncoder; 

    private final boolean encoderInverted; 
    private final double encoderOffsetRad; 

    public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReverse, boolean angleMotorReverse, 
         double encoderOffsetRad, boolean encoderInverted){

        this.encoderOffsetRad = encoderOffsetRad; 
        this.encoderInverted = encoderInverted; 


        driveMotor = new TalonFX(driveMotorID);  
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless); 
    
        angleMotor.setInverted(angleMotorReverse); 
        driveMotor.setInverted(driveMotorReverse); 

        angleEncoder = angleMotor.getEncoder(); 
        
        angleEncoder.setPositionConversionFactor(Constants.angleRot2Rad); 
        angleEncoder.setVelocityConversionFactor(Constants.angleRPM2RPS); 
        
        turningPidController = new PIDController(Constants.kPangle, Constants.kIangle, Constants.kDangle); 
        turningPidController.enableContinuousInput(-Math.PI , Math.PI);

        // magEncoder = new DigitalInput(encoderport);
        // encoder = new DutyCycle(magEncoder); 
        
        driveMotor.configOpenloopRamp(0.75);
        driveMotor.configClosedloopRamp(0.75); 
        driveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setIdleMode(IdleMode.kBrake); 
        
        resetEncoders();
    }

    public double getDrivePos(){
        return driveMotor.getSelectedSensorPosition(); 
    }

    public double getAnglePos(){
        return angleEncoder.getPosition(); 
    }

    public double getDriveVel(){
        return driveMotor.getSelectedSensorVelocity(); 
    }
    
    public double getAngleVel(){
        return angleEncoder.getVelocity(); 
    }
    
    //not using the mag encoder right now because we can't get them to work properly with the spark controllers 
    // public double getMagEncoder(){
    //     double angle = encoder.getOutput(); 
    //     angle *= 2 * Math.PI; 
    //     angle -= encoderOffsetRad; 
    //     return angle * (encoderInverted ? -1.0 : 1.0); 
    // }

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0); 
      //  angleEncoder.setPosition(getMagEncoder());
        angleEncoder.setPosition(0); 
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVel(), new Rotation2d(getAnglePos())); 
    }


    public void setDesiredState(SwerveModuleState state){

        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle); 
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.MAXDriveSpeed); 
        angleMotor.set(turningPidController.calculate(getAnglePos(), state.angle.getRadians()));
        SmartDashboard.putNumber("actual angle", getAnglePos() * 180 / Math.PI); 
        SmartDashboard.putNumber("desired angle", state.angle.getDegrees()); 
      //  SmartDashboard.putNumber("magencoder", getMagEncoder());
        //.putString("Swerve["+ getMagEncoder() + "] State", state.toString()); 

    }

    // public void flipModules(){
    //     angleMotor.set(turningPidController.calculate(getAnglePos(), Math.PI / 2));
    // }
    
    public void stop(){
        angleMotor.set(0);
        driveMotor.set(ControlMode.PercentOutput, 0);
    }


}
