// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class targetFollow extends Command {
  private final DriveSubsystem driveSubsystem;

  private int step;
  private final int SCORE_DISTANCE = 3;// limelight ta threshold for being "close enough" to score
  private final int LOAD_DISTANCE = 2; // distance in m to back up for load station from reef
  private final int LOAD_Y_OFFSET = 25;// lateral offset to back up to when loading
  private final int X_OFFSET = 15;//x offset to score on coral
  private long timer;

  private final double ROT_DEADBAND = 0.01;
  private final double XSPEED_DEADBAND = 0.01;
  private final double YSPEED_DEADBAND = 0.005;

  private final int DRIVE_TO_REEF =0;//step 0 in auto
  private final int FINAL_APPROACH = 2;// slow move to reef
  private final int SCORE = 1;//roller score on coral reef
  private final int MOVE_TO_LOAD = 3;//move from reef to loading station
  private final int WAIT_AT_LOAD = 4;//timed wait at loading station
  private final int DRIVE_OFF_START = 5;//move from start
  private int pipeline;
  /** Creates a new targetFollow. */
  public targetFollow(DriveSubsystem driveSubsystem, int startingpipeline) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    
    pipeline = startingpipeline;

    addRequirements(driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("targetFollow cmd started");
    step = DRIVE_TO_REEF;// steps in the program, starting at this step
    timer = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
LimelightHelpers.setPipelineIndex("limelight-front", pipeline);
    

    // this code section drives the robot to move toward an apriltag target
    double tx = LimelightHelpers.getTX("limelight-front");
    double ta = LimelightHelpers.getTA("limelight-front");
    boolean tv = LimelightHelpers.getTV("limelight-front");
    double tid = LimelightHelpers.getFiducialID("limelight-front");
    double[] targetpose =LimelightHelpers.getTargetPose_CameraSpace("limelight-front");//returns array of xyz adn pitch/roll/yaw position of target relative to robot
    
    double load_tx = LimelightHelpers.getTX("limelight-rear");
    double load_ta = LimelightHelpers.getTA("limelight-rear");
    boolean load_tv = LimelightHelpers.getTV("limelight-rear");
    double[] load_targetpose = LimelightHelpers.getTargetPose_CameraSpace("limelight-rear");

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putBoolean("tv", tv);
    /* SmartDashboard.putNumber("tid", tid);
    SmartDashboard.putNumber("targetpose0", targetpose[0]);
    SmartDashboard.putNumber("targetpose1", targetpose[1]);
    SmartDashboard.putNumber("targetpose2", targetpose[2]);
    SmartDashboard.putNumber("targetpose3", targetpose[3]); */
    SmartDashboard.putNumber("Step",step);
    SmartDashboard.putNumber("target angle", targetpose[4]);
  
    SmartDashboard.updateValues();
  
    
    switch(step){
      case DRIVE_OFF_START:
      driveSubsystem.drive(0.2, 0, 0, false);
      
      if(System.currentTimeMillis()>timer+4000){
        step = DRIVE_TO_REEF;
        timer = System.currentTimeMillis();
      }
      break;

      case DRIVE_TO_REEF://driving to reef with camera front alignment
      //rotational speed calculation based on
      //double rot = (-targetpose[4])/200;//25 is the limit view, normal angle based direction finding
      double rot = (X_OFFSET-tx)/100;//target centering center pixel based rotation
      if (rot < ROT_DEADBAND && rot > -ROT_DEADBAND){
        rot = 0;
      }
      // x (forward) speed calculation based on size of target in camera view
      double xSpeed = (SCORE_DISTANCE-ta)/(SCORE_DISTANCE*5);//ta=SCORE_DISTANCE is the target (bigger is closer), 0.2 is the speed limit, xSpeed is forward/reverse
      if(xSpeed < XSPEED_DEADBAND && xSpeed>-XSPEED_DEADBAND){
        xSpeed = 0;
      }
      if (xSpeed>0.15) {xSpeed=0.15;}//limit forward speed approaching target
      else if (xSpeed<-0.05) {xSpeed=-0.05;}//limit reverse speed adjusting back from target
      // y (lateral) speed calculation
      //double ySpeed = (-16-tx)/50;//- move right; center pixel based translation
      double ySpeed = targetpose[4]/300;//- move right; normal angle based translation
      if(ySpeed< YSPEED_DEADBAND&& ySpeed>-YSPEED_DEADBAND){
        ySpeed = 0;
      }else if (ySpeed>0.05){ySpeed=0.05;}//enforce lateral speed limit
      else if (ySpeed<-0.05){ySpeed=-0.05;}//enforce lateral speed limit
      if (ta<SCORE_DISTANCE*0.5){ySpeed = 0;}//only move lateral if close to target
      //no target detected
      if (!tv) {xSpeed = 0.05;rot = 0; ySpeed =0;}// what to do if no target is detected
      driveSubsystem.drive(xSpeed, 0, rot, false);
      if(Math.abs(X_OFFSET-tx)<1&&Math.abs(SCORE_DISTANCE-ta)<0.5){
        //&&Math.abs(targetpose[4])<8 //criteria for yaw control
        //check to switch steps
        step = FINAL_APPROACH;// switch to final movement
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case SCORE://scoring coral
      //need to hold elevator height at score height
      
      driveSubsystem.drive(0, 0, 0, false);
      if(System.currentTimeMillis()>timer+1000){// roll for 1000 ms
        //step = MOVE_TO_LOAD;// load next coral
        step = 99;//do nothing
        
        System.out.println("score done");
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case FINAL_APPROACH://final movement to reef
      driveSubsystem.drive(0.1, 0, 0, false);
      //move elevator to score level
      if(LimelightHelpers.getCurrentPipelineIndex("limelight-front")==0){
        
      //for center coral, goes upper level

      } else{
        
      //for side coral, lower level

      }
      
      if(System.currentTimeMillis()>timer+3000){// 2000 ms forward drive time
        step = 99;// switch to scoring
        
        timer = System.currentTimeMillis();
      }
      break;

      case MOVE_TO_LOAD:
      LimelightHelpers.setPipelineIndex("limelight-front", 2);//pipeline 2 is outer corner tags
      //System.out.println("move 2 load started");
      //rot = -tx/100;//25 is the limit view
      rot = -targetpose[4]/100;// align robot angle with normal vector of april tag
      if(rot <ROT_DEADBAND&&rot>-ROT_DEADBAND){rot = 0;}//rotation deadband to avoid twitching?
      xSpeed = (targetpose[2]-LOAD_DISTANCE)/(LOAD_DISTANCE*5)-0.01;//ta=SCORE_DISTANCE is the target (bigger is closer), 0.2 is the speed limit, xSpeed is forward/reverse
      if(xSpeed<XSPEED_DEADBAND&&xSpeed>-XSPEED_DEADBAND){
        xSpeed = 0;
      }
      if (xSpeed>0) {xSpeed=0;}// don't move forward when going to load
      else if (xSpeed<-0.2) {xSpeed=-0.2;}//limit backward speed to -0.2
      //ySpeed = targetpose[4]/300;//- move right
      ySpeed = (LOAD_Y_OFFSET-tx)/LOAD_Y_OFFSET;//offset the robot to one side
      if(ySpeed <YSPEED_DEADBAND && ySpeed >-YSPEED_DEADBAND){ySpeed=0;}//lateral deadband to avoid twitching
      else if (ySpeed > 0.2){ySpeed = 0.2;}//enforce lateral speed limit
      else if (ySpeed<-0.2){ySpeed=-0.2;}//enforce lateral speed limit
      if (ta<LOAD_DISTANCE/2){ySpeed = 0;}//only move lateral if close to target

      //no target detected code
      if (!tv) {xSpeed = -0.1;rot = 0; ySpeed =0;}// what to do if no target is detected, NECESSARY for initial move off wall
      
      driveSubsystem.drive(xSpeed, ySpeed, rot, false);
      if(Math.abs(tx)>(LOAD_Y_OFFSET-2)&&Math.abs(targetpose[4])<2&&Math.abs(targetpose[2])>=LOAD_DISTANCE&&tv||(System.currentTimeMillis()>timer+4000)){//need validity check tv
        //check to switch steps
        step = WAIT_AT_LOAD;// switch to final movement
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case WAIT_AT_LOAD:
      System.out.println("wait at load started");
      driveSubsystem.drive(0,0,0, false);
      
      if(System.currentTimeMillis()>timer+3000){// 1000 ms forward drive time
        step = DRIVE_TO_REEF;// switch to scoring
        timer = System.currentTimeMillis();
      }
      break;

      default:
      //do nothing by default
      break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.drive(0,0,0, false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
