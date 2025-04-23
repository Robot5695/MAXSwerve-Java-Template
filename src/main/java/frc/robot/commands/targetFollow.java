// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class targetFollow extends Command {
  private final DriveSubsystem driveSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final Elevator elevatorSubsystem;
  private int step;
  private final double SCORE_DISTANCE = 3;// limelight ta threshold for being "close enough" to score
  private final double LOAD_DISTANCE = 3; // distance in m to back up for load station from reef
  private final int LOAD_Y_OFFSET = 25;// lateral offset to back up to when loading
  private final int X_OFFSET = 17;//x offset to score on coral
  private long timer;//step timer
  private long autotimer;//overall auto timer
  //constants controlling speed and time in auto
  private final double MAX_FORWARD_SPD = 0.2;//max speed moving toward reef and load, tested value 0.15 (working), tested 0.3 (almost)
  private final int FINAL_APPROACH_DWELL = 2000;//ms for final approach
  private final int WAIT_AT_LOAD_DWELL = 2000;//ms for waiting at load
  private final int MOVE_TO_LOAD_TIMEOUT = 5000;//ms for exiting move to load
  private final int DRIVE_TO_REEF_TIMEOUT = 4000;//ms for exiting move to reef

  private final int SCORE_HEIGHT = 54;//elevator score height for reef

  private final double ROT_DEADBAND = 0.01;
  private final double XSPEED_DEADBAND = 0.01;
  private final double YSPEED_DEADBAND = 0.005;

  private final int DRIVE_TO_REEF =0;//step 0 in auto
  private final int FINAL_APPROACH = 2;// slow move to reef
  private final int SCORE = 1;//roller score on coral reef
  private final int MOVE_TO_LOAD = 3;//move from reef to loading station
  private final int WAIT_AT_LOAD = 4;//timed wait at loading station
  private final int DRIVE_OFF_START = 5;//move from start
  private final int ROTATE_TO_LOAD = 6;//back off reef, lower elevator, rotate to load

  private int piecesScored;//number of pieces scored
  private int pipeline;
  private int position;

  /** Creates a new targetFollow. */
  public targetFollow(DriveSubsystem driveSubsystem, RollerSubsystem rollerSubsystem, Elevator elevatorSubsystem, int startingposition) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    this.rollerSubsystem = rollerSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = startingposition;//tagfollower position 0 - left, 1 - center, 2 - right, 3 - demo

    /*
     * Pipelines
     * center tags 10,21
     * inner corner tags 9,11,20,22
     * all reef tags 6-11, 17-22
     * loading tags 1,2,12,13
     * 
     * 0: center_tags
     * 1: inner_corners
     * 2: outer_corners (6,8,17,19)
     * 3: not defined on LL
     * 4: not defined on LL
     */
    //tagfollower position 0 - left, 1 - center, 2 - right
    if(startingposition==1){
      //center
      pipeline = 0;

    }else if (startingposition==3){
pipeline=3;//demo pipeline
    }else{
      //left or right
      pipeline = 1;
    }
    

    addRequirements(driveSubsystem);
    addRequirements(rollerSubsystem);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("targetFollow cmd started");
    step = DRIVE_TO_REEF;// steps in the program, starting at this step
    timer = System.currentTimeMillis();
    autotimer = System.currentTimeMillis();
    LimelightHelpers.setPipelineIndex("limelight-front", pipeline);
    piecesScored=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

    // this code section drives the robot to move toward an apriltag target
    double tx = LimelightHelpers.getTX("limelight-front");
    double ta = LimelightHelpers.getTA("limelight-front");
    boolean tv = LimelightHelpers.getTV("limelight-front");
    double tid = LimelightHelpers.getFiducialID("limelight-front");
    double[] targetpose =LimelightHelpers.getTargetPose_CameraSpace("limelight-front");//returns array of xyz adn pitch/roll/yaw position of target relative to robot
    
    LimelightHelpers.setPipelineIndex("limelight-rear", 3);//pipeline 3 is loading tags
    double load_tx = LimelightHelpers.getTX("limelight-rear");
    double load_ta = LimelightHelpers.getTA("limelight-rear");
    boolean load_tv = LimelightHelpers.getTV("limelight-rear");
    double[] load_targetpose = LimelightHelpers.getTargetPose_CameraSpace("limelight-rear");

    //SmartDashboard.putNumber("tx", tx);
    //SmartDashboard.putNumber("ta", ta);
    //SmartDashboard.putBoolean("tv", tv);
    /* SmartDashboard.putNumber("tid", tid);
    SmartDashboard.putNumber("targetpose0", targetpose[0]);
    SmartDashboard.putNumber("targetpose1", targetpose[1]);
    SmartDashboard.putNumber("targetpose2", targetpose[2]);
    SmartDashboard.putNumber("targetpose3", targetpose[3]); */
    SmartDashboard.putNumber("Step",step);
    //SmartDashboard.putNumber("target angle", targetpose[4]);
  
    SmartDashboard.updateValues();
  
    
    switch(step){
      case DRIVE_OFF_START:
      //not used
      driveSubsystem.drive(0.2, 0, 0, false);
      elevatorSubsystem.setTarget(0);
      if(System.currentTimeMillis()>timer+4000){
        step = DRIVE_TO_REEF;
        timer = System.currentTimeMillis();
      }
      break;

      case DRIVE_TO_REEF://driving to reef with camera front alignment
      //rotational speed calculation based on
      //double rot = (-targetpose[4])/200;//25 is the limit view, normal angle based direction finding
      double rot = 0;
      //if the robot has scored multiple pieces OR is coming from the left, use the right position
      if(piecesScored>1 || position ==0){
         rot = (-X_OFFSET-tx)/100;//target centering center pixel based rotation - right pole
      }else{
         rot = (X_OFFSET-tx)/100;//target centering center pixel based rotation - left pole
      }
      
      if (rot < ROT_DEADBAND && rot > -ROT_DEADBAND){
        rot = 0;
      }
      // x (forward) speed calculation based on size of target in camera view
      //tested working values (SCORE_DISTANCE-ta)/(SCORE_DISTANCE*5)
      double xSpeed = (SCORE_DISTANCE-ta)/(SCORE_DISTANCE*2.5);//ta=SCORE_DISTANCE is the target (bigger is closer), 0.2 is the speed limit, xSpeed is forward/reverse
      if(xSpeed < XSPEED_DEADBAND && xSpeed>-XSPEED_DEADBAND){
        xSpeed = 0;
      }
      if (xSpeed>MAX_FORWARD_SPD) {xSpeed=MAX_FORWARD_SPD;}//limit forward speed approaching target
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
      //tested default speed 0.05
      if (!tv) {
        if (pipeline!=3){
          rot = 0;
          xSpeed = 0.1;
        }else{
          rot = 0.1;
          xSpeed = 0;//demo program
        }
        
         

        ySpeed =0;}// what to do if no target is detected

      driveSubsystem.drive(xSpeed, 0, rot, false);
      if(Math.abs(X_OFFSET-tx)<1&&Math.abs(SCORE_DISTANCE-ta)<0.5||System.currentTimeMillis()>timer+DRIVE_TO_REEF_TIMEOUT){
        //&&Math.abs(targetpose[4])<8 //criteria for yaw control
        //check to switch steps
        
        if(pipeline!=3){
          step = FINAL_APPROACH;// switch to final movement
        }
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case SCORE://scoring coral
      //need to hold elevator height at score height
      rollerSubsystem.runRoller(0, 0.5);
      driveSubsystem.drive(0, 0, 0, false);
      
      if(System.currentTimeMillis()>timer+1000){// roll for 1000 ms
        //step = MOVE_TO_LOAD;// load next coral
        step = ROTATE_TO_LOAD;//do nothing
        piecesScored++;
        rollerSubsystem.runRoller(0, 0);
        elevatorSubsystem.setTarget(0);
        //for center scoring robot, stop at this point
        if(position==1){
//tagfollower position 0 - left, 1 - center, 2 - right
step = 99;//no further steps
        }

        System.out.println("score done");
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case FINAL_APPROACH://final movement to reef
      driveSubsystem.drive(0.15, 0, 0, false);
      //move elevator to score level
      if(LimelightHelpers.getCurrentPipelineIndex("limelight-front")==0){
        elevatorSubsystem.setTarget(SCORE_HEIGHT);
      //for center coral, goes upper level

      } else{
        elevatorSubsystem.setTarget(SCORE_HEIGHT);
      //for side coral, lower level

      }
      
      if(System.currentTimeMillis()>timer+FINAL_APPROACH_DWELL){// 2000 ms forward drive time
        step = SCORE;// switch to scoring
        
        timer = System.currentTimeMillis();
      }
      break;

      case ROTATE_TO_LOAD:
      elevatorSubsystem.setTarget(0);
      if(piecesScored>1){
        //subsequent scores back up directly without rotating
        driveSubsystem.drive(-0.1, 0, 0, false);
      }else{
        //need to determine left vs right rotation for first backup
        if(position==0){
          //left side back up counter-clockwise
          driveSubsystem.drive(-0.1, 0, 0.3, false);
        }else if (position == 2){
          //right side back up clockwise
          driveSubsystem.drive(-0.1, 0, -0.3, false);
        }
        
      }
      
      if(System.currentTimeMillis()>timer+1000){
        step=MOVE_TO_LOAD;
        timer=System.currentTimeMillis();
        LimelightHelpers.setPipelineIndex("limelight-rear", 3);//pipeline 3 is loading tags
      }
      break;

      case MOVE_TO_LOAD:
      LimelightHelpers.setPipelineIndex("limelight-front", 2);//pipeline 2 is outer corner tags
      elevatorSubsystem.setTarget(0);
      //System.out.println("move 2 load started");
      rot = -load_tx/100;//25 is the limit view
      //rot = -targetpose[4]/100;// align robot angle with normal vector of april tag
      if(rot <ROT_DEADBAND&&rot>-ROT_DEADBAND){rot = 0;}//rotation deadband to avoid twitching?

      xSpeed = (LOAD_DISTANCE-load_ta)/(LOAD_DISTANCE);//ta=SCORE_DISTANCE is the target (bigger is closer), 0.2 is the speed limit, xSpeed is forward/reverse
      SmartDashboard.putNumber("xspeed", xSpeed);
      SmartDashboard.putNumber("load_ta", load_ta);
      if(xSpeed>MAX_FORWARD_SPD){
        xSpeed=MAX_FORWARD_SPD;//set maximum speed moving toward load
      }else if (xSpeed<-0.2){
        xSpeed=-0.2;
      }
     

      //ySpeed = targetpose[4]/300;//- move right
      ySpeed = (LOAD_Y_OFFSET-tx)/LOAD_Y_OFFSET;//offset the robot to one side
      if(ySpeed <YSPEED_DEADBAND && ySpeed >-YSPEED_DEADBAND){ySpeed=0;}//lateral deadband to avoid twitching
      else if (ySpeed > 0.2){ySpeed = 0.2;}//enforce lateral speed limit
      else if (ySpeed<-0.2){ySpeed=-0.2;}//enforce lateral speed limit
      if (ta<LOAD_DISTANCE/2){ySpeed = 0;}//only move lateral if close to target

      //no target detected code
      if (!load_tv) {xSpeed = 0.15;rot = 0; ySpeed =0;}// what to do if no target is detected, NECESSARY for initial move off wall
      
      driveSubsystem.drive(-xSpeed, 0, rot, false);
      if(Math.abs(load_tx)<0.5&&Math.abs(LOAD_DISTANCE-load_ta)<0.5||System.currentTimeMillis()>timer+MOVE_TO_LOAD_TIMEOUT){//need validity check tv
        //check to switch steps
        step = WAIT_AT_LOAD;// switch to final movement
        timer = System.currentTimeMillis();
        break;
      }
      break;

      case WAIT_AT_LOAD:
      System.out.println("wait at load started");
      elevatorSubsystem.setTarget(0);
      driveSubsystem.drive(-0.1,0,0, false);
      rollerSubsystem.runRoller(0, 0.2);
      if(System.currentTimeMillis()>timer+WAIT_AT_LOAD_DWELL){// 1000 ms forward drive time
        step = DRIVE_TO_REEF;// switch to scoring
        timer = System.currentTimeMillis();
        rollerSubsystem.runRoller(0, 0);
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
    rollerSubsystem.runRoller(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
