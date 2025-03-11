package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReef extends Command {
    private PIDController xController, yController, rController;
    private boolean isRight;
    private Timer dontSeeTagTimer, stopTimer;
    private DriveSubsystem drivebase;
    private double tagID = -1; 

    public AlignToReef(boolean isRight, DriveSubsystem drivebase){
        xController = new PIDController(AutoConstants.x_reef_alignment_Pvalue, 0, 0);
        yController = new PIDController(AutoConstants.y_reef_alignment_Pvalue, 0, 0);
        rController = new PIDController(AutoConstants.r_reef_alignment_Pvalue, 0, 0);
        this.isRight = isRight;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rController.setSetpoint(AutoConstants.rotational_setpoint);
        rController.setTolerance(AutoConstants.rotational_tolerance);

        xController.setSetpoint(AutoConstants.x_setpoint);
        xController.setTolerance(AutoConstants.x_tolerance);

        rController.setSetpoint(AutoConstants.y_setpoint);
        rController.setTolerance(AutoConstants.y_tolerance);

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("")== tagID){
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            double xSpeed = xController.calculate(positions[2]);
            double ySpeed = yController.calculate(positions[0]);
            double rotationValue = -rController.calculate(positions[4]);

            drivebase.drive(xSpeed, ySpeed, rotationValue, false);

            if (!rController.atSetpoint() || !xController.atSetpoint() || !yController.atSetpoint()){
                stopTimer.reset();
            }
            else {
                drivebase.drive(0,0, 0, false);
            }

        }
    }


    @Override
    public void end(boolean interuppted) {
        drivebase.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return this.dontSeeTagTimer.hasElapsed(1) || stopTimer.hasElapsed(0.3);
    }
}