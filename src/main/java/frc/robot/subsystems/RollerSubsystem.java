// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


/** Class to run the rollers over CAN */
public class RollerSubsystem extends SubsystemBase {
  private TalonSRX RollerMotor = new TalonSRX(RollerConstants.kRollerMotorCanId);
  //need to add spark max motor controler for final design

  public RollerSubsystem() {
    RollerMotor.configContinuousCurrentLimit(RollerConstants.kRollerCurrentLimit);
  }

  public void setRollerSpeed(double MotorSpeed){
    RollerMotor.set(ControlMode.PercentOutput,MotorSpeed);
  }

  public void stopRollerMotor(){
    RollerMotor.set(ControlMode.PercentOutput,0);
  }

  public void runRoller(double forward, double reverse){
    RollerMotor.set(ControlMode.PercentOutput,forward - reverse);
  } 


  @Override
  public void periodic() {
  }

}