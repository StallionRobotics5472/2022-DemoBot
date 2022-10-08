// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    private final static SwerveModule frontLeft = new SwerveModule(
        Constants.frontLeft, 
        Constants.a_frontLeft, 
        Constants.driveMotorForward, 
        Constants.angleMotorFoward, 
        Constants.encoderOffsetRad, false); 

    private final static SwerveModule frontRight = new SwerveModule(
        Constants.frontRight, 
        Constants.a_frontRight, 
        Constants.driveMotorForward, 
        Constants.angleMotorFoward, 
        Constants.encoderOffsetRad, false); 

    private final static SwerveModule backLeft = new SwerveModule(
        Constants.backLeft, 
        Constants.a_backLeft, 
        Constants.driveMotorForward, 
        Constants.angleMotorFoward, 
        Constants.encoderOffsetRad, false); 
        
    private final static SwerveModule backRight = new SwerveModule(
        Constants.backRight, 
        Constants.a_backRight, 
        Constants.driveMotorForward, 
        Constants.angleMotorFoward, 
        Constants.encoderOffsetRad, false); 
        private double keepAngle = 0.0; // Double to store the current target keepAngle in radians
        private double timeSinceRot = 0.0; // Double to store the time since last rotation command
        private double lastRotTime = 0.0; // Double to store the time of the last rotation command
        private double timeSinceDrive = 0.0; // Double to store the time since last translation command
        private double lastDriveTime = 0.0; // Double to store the time of the last translation command
        private final Timer keepAngleTimer = new Timer(); // Creates timer used in the perform keep angle function
        private static AHRS ahrs = new AHRS(SPI.Port.kMXP);
        private final PIDController m_keepAnglePID = new PIDController(Constants.kKeepAnglePID[0],
      Constants.kKeepAnglePID[1], Constants.kKeepAnglePID[2]);
        private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.kDriveKinematics,
          ahrs.getRotation2d());

          public Drivetrain() {
            keepAngleTimer.reset();
            keepAngleTimer.start();
            m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
            ahrs.reset();
            m_odometry.resetPosition(new Pose2d(), ahrs.getRotation2d().times(-1.0));
          }
          @SuppressWarnings("ParameterName")
          public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
            rot = performKeepAngle(xSpeed, ySpeed, rot); // Calls the keep angle function to update the keep angle or rotate
                                                         // depending on driver input
        
            // SmartDashboard.putNumber("xSpeed Commanded", xSpeed);
            // SmartDashboard.putNumber("ySpeed Commanded", ySpeed);
        
            // creates an array of the desired swerve module states based on driver command
            // and if the commands are field relative or not
            var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        
            // normalize wheel speeds so all individual states are scaled to achievable
            // velocities
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAXDriveSpeed);
        
            setModuleState(swerveModuleStates);
          }
     //   private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0)); 

    // public void zeroHeading(){
    //     gyro.reset();
    // }
    // public double getHeading(){
    //     return Math.IEEEremainder(gyro.getAngle(), 360); 
    // }
    // public Rotation2d getRotation2d() {  
    //     return Rotation2d.fromDegrees(getHeading());
    // }
    // public Pose2d getPose() {
    //     return odometer.getPoseMeters();
    // }
    // public void resetOdometry(Pose2d pose) {
    //     odometer.resetPosition(pose, getRotation2d());
    // }
    
    @Override
    public void periodic() {
        // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        //         backRight.getState());
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        updateOdometry();
        getPose();
    }



    public static void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleState(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAXDriveSpeed);
        frontLeft.setDesiredState(desiredStates[0]); 
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public void updateOdometry() {
        m_odometry.update(ahrs.getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
            backRight.getState());
      }

      public Rotation2d getGyro() {
        return ahrs.getRotation2d();
      }
      public Pose2d getPose() {
        Pose2d pose = m_odometry.getPoseMeters();
        Translation2d position = pose.getTranslation();
        SmartDashboard.putNumber("Robot X", position.getX());
        SmartDashboard.putNumber("Robot Y", position.getY());
        SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());
        return m_odometry.getPoseMeters();
      }
      public void resetOdometry(Pose2d pose) {
        ahrs.reset();
        ahrs.setAngleAdjustment(pose.getRotation().getDegrees());
        keepAngle = getGyro().getRadians();
        m_odometry.resetPosition(pose, ahrs.getRotation2d().times(-1.0));
      }
    
      public void setPose(Pose2d pose) {
        m_odometry.resetPosition(pose, ahrs.getRotation2d().times(-1.0));
        keepAngle = getGyro().getRadians();
      }   
      public void resetOdometry(Rotation2d angle) {
        Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
        ahrs.reset();
        ahrs.setAngleAdjustment(angle.getDegrees());
        keepAngle = getGyro().getRadians();
        m_odometry.resetPosition(pose, ahrs.getRotation2d().times(-1.0));
      }
      public ChassisSpeeds getChassisSpeed() {
        return Constants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
            backLeft.getState(),
            backRight.getState());
      }
      private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
        double output = rot; // Output should be set to the input rot command unless the Keep Angle PID is
                             // called
        if (Math.abs(rot) >= Constants.kMinRotationCommand) { // If the driver commands the robot to rotate set the
                                                                   // last rotate time to the current time
          lastRotTime = keepAngleTimer.get();
        }
        if (Math.abs(xSpeed) >= Constants.kMinTranslationCommand
            || Math.abs(ySpeed) >= Constants.kMinTranslationCommand) { // if driver commands robot to translate set the
                                                                            // last drive time to the current time
          lastDriveTime = keepAngleTimer.get();
        }
        timeSinceRot = keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
        timeSinceDrive = keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive time
        if (timeSinceRot < 0.5) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                                  // move to finish
          keepAngle = getGyro().getRadians();
        } else if (Math.abs(rot) < Constants.kMinRotationCommand && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                                                  // until 0.75s after drive
                                                                                                  // command stops to combat
                                                                                                  // decel drift
          output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle); // Set output command to the result of the
                                                                                // Keep Angle PID
        }
        return output;
      }
    
      public void updateKeepAngle() {
        keepAngle = getGyro().getRadians();
      }
     


  
}
