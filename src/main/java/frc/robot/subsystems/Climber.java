// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



/** Class to run the Climbers over CAN */
public class Climber extends SubsystemBase {
  //private TalonSRX ClimberMotor = new TalonSRX(ClimberConstants.kClimberMotorCanId);
  private SparkMax ClimberMotor; //need to set up sparkmax motor contClimber ID
  private final SparkMaxConfig ClimberMotorConfig;
  //need to add spark max motor controler for final design

  public Climber() {
    //ClimberMotor.configContinuousCurrentLimit(ClimberConstants.kClimberCurrentLimit);
    ClimberMotor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
    ClimberMotorConfig =  new SparkMaxConfig();
    ClimberMotorConfig.smartCurrentLimit(ClimberConstants.kClimberMotorStallCurrent);
    ClimberMotorConfig.idleMode(IdleMode.kBrake);//brake climber motor to hold
    ClimberMotor.configure(ClimberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ClimberMotor.set(0);

  }

  public void setClimberSpeed(double MotorSpeed){
    ClimberMotor.set(MotorSpeed);
  }

  public void stopClimberMotor(){
    ClimberMotor.set(0);
  }

  public void runClimber(double forward, double reverse){
    ClimberMotor.set(forward - reverse);
  } 


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber current", ClimberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber Speed", ClimberMotor.get());
  }

}