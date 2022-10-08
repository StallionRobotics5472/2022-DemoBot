// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.MathUtils;

public class DriveByController extends CommandBase {

    private final Drivetrain swerveSubSystem;
    private final XboxController m_controller;
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(Constants.kTranslationSlew);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(Constants.kTranslationSlew);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(Constants.kRotationSlew);
  private boolean fieldOrient = false;
  //  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction; 

  //  private final Supplier<Boolean> fieldOrientedFunction;

    

  /*public SwerveJoystickCMD(SwerveSubSystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
      this.swerveSubSystem = swerveSubsystem; 
      this.xSpdFunction = xSpdFunction; 
      this.ySpdFunction = ySpdFunction; 
      this.turningSpdFunction = turningSpdFunction; 
      this.fieldOrientedFunction = fieldOrientedFunction;
      addRequirements(swerveSubsystem);
  }
*/
public DriveByController(Drivetrain drive, XboxController controller) {
  swerveSubSystem = drive; // Set the private member to the input drivetrain
  m_controller = controller; // Set the private member to the input controller
  addRequirements(swerveSubSystem); // Because this will be used as a default command, add the subsystem which will
                                 // use this as the default
}

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    swerveSubSystem.drive(m_slewX.calculate(
        -inputTransform(m_controller.getLeftY()))
        * Constants.MAXDriveSpeed,
        m_slewY.calculate(
            inputTransform(m_controller.getLeftX()))
            * Constants.MAXDriveSpeed,
        m_slewRot.calculate(inputTransform(m_controller.getRightX()))
            * Constants.kMaxAngularSpeed,
        fieldOrient);

        SmartDashboard.putBoolean("DrivingByController", true);
  }
 /* public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get(); 

    xSpeed = Math.abs(xSpeed) > Constants.Deadband ? xSpeed : 0; 
    ySpeed = Math.abs(ySpeed) > Constants.Deadband ? ySpeed : 0; 
    turningSpeed = Math.abs(turningSpeed) > Constants.Deadband ? turningSpeed : 0; 
    
    ChassisSpeeds chassisSpeeds;
    /* if (fieldOrientedFunction.get()) {
       // Relative to field
       chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
               xSpeed, ySpeed, turningSpeed);
     } else {
       // Relative to robot
       chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
     }
     */
   /* chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 

    swerveSubSystem.setModuleState(moduleStates);
    
  }
*/
  @Override
  public void end(boolean interrupted) {
    Drivetrain.stopModules();
  }
  public void changeFieldOrient() {
    if (fieldOrient) {
      fieldOrient = false;
    } else {
      fieldOrient = true;
    }
  }
  private double inputTransform(double input) {
    return MathUtils.singedSquare(MathUtils.applyDeadband(input));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
