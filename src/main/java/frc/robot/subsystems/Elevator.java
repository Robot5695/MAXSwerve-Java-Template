// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;

/** Class to run the rollers over CAN */
public class Elevator extends SubsystemBase {
  private final SparkMax elevator1;
  private final SparkMax elevator2;
  private final SparkMaxConfig elevator1config, elevator2config;
  private final RelativeEncoder elevatorEncoder;
  private double target;
  private final ProfiledPIDController m_controller;
  private final ElevatorFeedforward elevatorFeedforward;

  private final SparkClosedLoopController elevatorClosedLoopController;

  public Elevator() {
    elevator1= new SparkMax(ElevatorConstants.elevator1CanId, MotorType.kBrushless);
    elevator2= new SparkMax(ElevatorConstants.elevator2CanId, MotorType.kBrushless);

    m_controller = new ProfiledPIDController(
      ElevatorConstants.kElevatorKp, 
      ElevatorConstants.kElevatorKi, 
      ElevatorConstants.kElevatorKd, 
      new TrapezoidProfile.Constraints(ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration)
    );
    elevatorFeedforward = new ElevatorFeedforward(
      ElevatorConstants.kElevatorkS, 
      ElevatorConstants.kElevatorkG,
      ElevatorConstants.kElevatorkV,
      ElevatorConstants.kElevatorkA
      );

    elevatorEncoder = elevator1.getEncoder();
  
    
    elevator1config = new SparkMaxConfig();
    elevator2config = new SparkMaxConfig();
    elevator1config.inverted(false);
    elevator2config.follow(elevator1, true);
    elevator1config.idleMode(IdleMode.kBrake);
    elevator2config.idleMode(IdleMode.kBrake);
    elevator1config.smartCurrentLimit(35,35);//set current limits
    elevator2config.smartCurrentLimit(35,35);//set current limits
    elevator1config.closedLoop.pid(2,0,0);//PID values for elevator 1
    elevatorClosedLoopController = elevator1.getClosedLoopController();
    elevator1.configure(elevator1config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);//set elevator 1 configuration
    elevator2.configure(elevator2config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);//set follower
    elevatorEncoder.setPosition(0);
    target = elevatorEncoder.getPosition();
    elevatorClosedLoopController.setReference(target, ControlType.kPosition);
  }

  public void setTarget(double newTarget){
    if(newTarget>target){
      //moving up control
      elevator1config.closedLoop.pid(0.3,0,0);//positive D could help "soften" upward approach to target
    }else if(newTarget<target){
      //moving down
      elevator1config.closedLoop.pid(0.3,0,0);//positive D could help "soften" downward approach to target
    }

    target = newTarget;
    //set low/high limits
    if (target<0){
      target = 0;
    }
    if (target>57){
      target = 57;
    }
    //command the controller to drive motor towards the target
    elevatorClosedLoopController.setReference(target, ControlType.kPosition);
  }

  public double getTarget(){
    return target;
  }

  public double getPosition(){
    return elevatorEncoder.getPosition();

  }

  /* 
  //possible command to go to L2, untested
  public void goToL2(){
    m_controller.setGoal(ElevatorConstants.l2);

    double pidOutput = m_controller.calculate(elevatorEncoder.getPosition());
    double feedforwardouput = elevatorFeedforward.calculate(m_controller.getSetpoint().velocity);
    
    elevator1.setVoltage(pidOutput+feedforwardouput);

  }*/

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    
    SmartDashboard.putNumber("Elevator Target", target);
    SmartDashboard.putNumber("elevator1 current", elevator1.getOutputCurrent());
    SmartDashboard.putNumber("elevator2 current", elevator2.getOutputCurrent());
  }

}