// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class targetFollow extends Command {
  private final DriveSubsystem driveSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private int step;
  private final int SCORE_DISTANCE = 20;// limelight ta threshold for being "close enough" to score
  private final int LOAD_DISTANCE = 5; // distance to back up for load station from reef 
  private long timer;

  private final int DRIVE_TO_REEF =0;//step 0 in auto
  private final int FINAL_APPROACH = 2;// slow move to reef
  private final int SCORE = 1;//roller score on coral reef
  private final int MOVE_TO_LOAD = 3;//move from reef to loading station
  private final int WAIT_AT_LOAD = 4;//timed wait at loading station

  /** Creates a new targetFollow. */
  public targetFollow(DriveSubsystem driveSubsystem, RollerSubsystem rollerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    this.rollerSubsystem = rollerSubsystem;

    addRequirements(driveSubsystem);
    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("targetFollow cmd started");
    step = DRIVE_TO_REEF;// steps in the program, starting at zero (drive to reef)
    timer = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

    // this code section drives the robot to move toward an apriltag target
    double tx = LimelightHelpers.getTX("");
    double ta = LimelightHelpers.getTA("");
    boolean tv = LimelightHelpers.getTV("");
    double[] targetpose =LimelightHelpers.getTargetPose_CameraSpace("");//returns array of xyz adn pitch/roll/yaw position of target relative to robot
    
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putBoolean("tv", tv);
    SmartDashboard.putNumber("targetpose0", targetpose[0]);
    SmartDashboard.putNumber("targetpose1", targetpose[1]);
    SmartDashboard.putNumber("targetpose2", targetpose[2]);
    SmartDashboard.putNumber("targetpose3", targetpose[3]);
    SmartDashboard.putNumber("Step",step);
    SmartDashboard.putNumber("target angle", targetpose[4]);
  
    SmartDashboard.updateValues();
  
    
    switch(step){
      case DRIVE_TO_REEF://driving to reef
      double rot = -tx/100;//25 is the limit view
      double xSpeed = -(ta-SCORE_DISTANCE)/(SCORE_DISTANCE*5);//ta=SCORE_DISTANCE is the target (bigger is closer), 0.2 is the speed limit, xSpeed is forward/reverse
      if (xSpeed>0.2) {xSpeed=0.2;}
      else if (xSpeed<-0.05) {xSpeed=-0.05;}
      double ySpeed = targetpose[4]/300;//- move right
      if (ta<SCORE_DISTANCE/2){ySpeed = 0;}//only move lateral if close to target
      if (!tv) {xSpeed = 0;rot = -0.1; ySpeed =0;}// what to do if no target is detected
      driveSubsystem.drive(xSpeed, ySpeed, rot, false);
      if(Math.abs(tx)<1&&Math.abs(targetpose[4])<2&&Math.abs(ta)>SCORE_DISTANCE){
        //check to switch steps
        step = FINAL_APPROACH;// switch to final movement
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case SCORE://scoring coral
      rollerSubsystem.runRoller(0, 0.3);
      driveSubsystem.drive(0, 0, 0, false);
      if(System.currentTimeMillis()>timer+1000){// roll for 1000 ms
        step = MOVE_TO_LOAD;// end
        rollerSubsystem.runRoller(0, 0);
        System.out.println("score done");
        break;
      }
      break;

      case FINAL_APPROACH://final movement to reef
      driveSubsystem.drive(0.05, 0, 0, false);
      if(System.currentTimeMillis()>timer+1000){// 1000 ms forward drive time
        step = SCORE;// switch to scoring
        timer = System.currentTimeMillis();
      }
      break;

      case MOVE_TO_LOAD:
      System.out.println("move 2 load started");
      rot = -tx/100;//25 is the limit view
       xSpeed = -(ta-LOAD_DISTANCE)/(LOAD_DISTANCE*5);//ta=SCORE_DISTANCE is the target (bigger is closer), 0.2 is the speed limit, xSpeed is forward/reverse
      if (xSpeed>0.05) {xSpeed=0.05;}
      else if (xSpeed<-0.2) {xSpeed=-0.2;}
      ySpeed = targetpose[4]/300;//- move right
      if (ta<LOAD_DISTANCE/2){ySpeed = 0;}//only move lateral if close to target
      if (!tv) {xSpeed = -0.1;rot = 0; ySpeed =0;}// what to do if no target is detected
      driveSubsystem.drive(xSpeed, ySpeed, rot, false);
      if(Math.abs(tx)<1&&Math.abs(targetpose[4])<1&&Math.abs(ta)<LOAD_DISTANCE){//need validity check tv
        //check to switch steps
        step = WAIT_AT_LOAD;// switch to final movement
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case WAIT_AT_LOAD:
      System.out.println("wait at load started");
      driveSubsystem.drive(0,0,0, false);
      rollerSubsystem.runRoller(0, 0);
      if(System.currentTimeMillis()>timer+3000){// 1000 ms forward drive time
        step = DRIVE_TO_REEF;// switch to scoring
        timer = System.currentTimeMillis();
      }
      break;

      default:

      break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.drive(0,0,0, false);
    rollerSubsystem.runRoller(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
