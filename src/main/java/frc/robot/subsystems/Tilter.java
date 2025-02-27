// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TilterConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



/** Class to run the Tilters over CAN */
public class Tilter extends SubsystemBase {
  //private TalonSRX TilterMotor = new TalonSRX(TilterConstants.kTilterMotorCanId);
  private SparkMax TilterMotor; //need to set up sparkmax motor contTilter ID
  private final SparkMaxConfig TilterMotorConfig;
  //need to add spark max motor controler for final design

  public Tilter() {
    //TilterMotor.configContinuousCurrentLimit(TilterConstants.kTilterCurrentLimit);
    TilterMotor = new SparkMax(TilterConstants.kTilterMotorCanId, MotorType.kBrushless);
    TilterMotorConfig =  new SparkMaxConfig();
    TilterMotorConfig.smartCurrentLimit(TilterConstants.kTilterMotorStallCurrent);
    TilterMotorConfig.idleMode(IdleMode.kBrake);//brake Tilter motor to hold
    TilterMotor.configure(TilterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    TilterMotor.set(0);

  }

  public void setTilterSpeed(double MotorSpeed){
    TilterMotor.set(MotorSpeed);
  }

  public void stopTilterMotor(){
    TilterMotor.set(0);
  }

  public void runTilter(double forward, double reverse){
    TilterMotor.set(forward - reverse);
  } 


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tilter current", TilterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Tilter Speed", TilterMotor.get());
  }

}