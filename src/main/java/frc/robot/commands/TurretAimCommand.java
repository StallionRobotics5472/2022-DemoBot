package frc.robot.commands;

import java.util.concurrent.ConcurrentLinkedQueue;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.RobotContainer;
// import frc.robot.Subsystems.LidarSubsystem;
// import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.subsystems.Drivetrain;


public class TurretAimCommand extends CommandBase {

    /*
     * Approaches a target given the target is within the camera's fov
     */

    private boolean finished;
    private Limelight limelight;

    // private TurretSubsystem turret;
    // private LidarSubsystem lidar;
    private Drivetrain drive;
    private final SlewRateLimiter m_slewRot = new SlewRateLimiter(Constants.kRotationSlew);


    public TurretAimCommand() {
        // addRequirements(Robot.turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("LimelightAim", false);
        finished = false;
        // turret = Robot.turret
        drive = Robot.drive;
        limelight = Robot.limelight;
        limelight.setWallTargetPipeline();

        // turret.turret.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void execute() {

        // SmartDashboard.putNumber("LIDAR", lidar.getDistance());

        double turn;

        double horizontalError = limelight.getHorizontalAngle();
        SmartDashboard.putNumber("ty", limelight.getHorizontalAngle());
        turn = horizontalError * -0.06;

        SmartDashboard.putNumber("Turret Rotation", turn);
        if (limelight.getTargetArea() <= 1e-4) {
            // || limelight.isFrozen()) {
            turn = 0;
        }

        else {
            if (Math.abs(limelight.getHorizontalAngle()) < 0.5) {
                SmartDashboard.putBoolean("LimelightAim", true);

            } else if (Math.abs(limelight.getHorizontalAngle()) < 30)
                turn *= 3;
            else if (Math.abs(limelight.getHorizontalAngle()) < 20)
                turn *= 4;
            else if (Math.abs(limelight.getHorizontalAngle()) < 10)
                turn *= 5;
            else if (Math.abs(limelight.getHorizontalAngle()) < 5)
                turn *= 6;
            else {
                turn *= 2;
            }
        }
        drive.drive(0,0, m_slewRot.calculate(-(turn)), false);

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.disableWallTargetPipeline();
        // turret.turret.setIdleMode(IdleMode.kBrake);
    }
}