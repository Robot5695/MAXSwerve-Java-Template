// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



/** Class to run the rollers over CAN */
public class RollerSubsystem extends SubsystemBase {
  //private TalonSRX RollerMotor = new TalonSRX(RollerConstants.kRollerMotorCanId);
  private SparkMax RollerMotor; //need to set up sparkmax motor controller ID
  private final SparkMaxConfig rollerMotorConfig;
  //need to add spark max motor controler for final design

  public RollerSubsystem() {
    //RollerMotor.configContinuousCurrentLimit(RollerConstants.kRollerCurrentLimit);
    RollerMotor = new SparkMax(RollerConstants.kRollerMotorCanId, MotorType.kBrushless);
    rollerMotorConfig =  new SparkMaxConfig();
    rollerMotorConfig.smartCurrentLimit(35);
    rollerMotorConfig.idleMode(IdleMode.kCoast);
    RollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RollerMotor.set(0);

  }

  public void setRollerSpeed(double MotorSpeed){
    RollerMotor.set(MotorSpeed);
  }

  public void stopRollerMotor(){
    RollerMotor.set(0);
  }

  public void runRoller(double forward, double reverse){
    RollerMotor.set(forward - reverse);
  } 


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roller current", RollerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Roller Speed", RollerMotor.get());
  }

}