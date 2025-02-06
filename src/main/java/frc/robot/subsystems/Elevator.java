// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ElevatorConstants;

/** Class to run the rollers over CAN */
public class Elevator extends SubsystemBase {
  private final SparkMax elevator1;
  private final SparkMax elevator2;
  private final RelativeEncoder elevatorEncoder;


  private final SparkClosedLoopController elevatorClosedLoopController;
  //need to add spark max motor controler for final design

  public Elevator() {
     elevator1= new SparkMax(ElevatorConstants.elevator1CanId, MotorType.kBrushless);
     elevator2= new SparkMax(ElevatorConstants.elevator2CanId, MotorType.kBrushless);
    elevatorEncoder = elevator1.getEncoder();
  
    elevatorClosedLoopController = elevator1.getClosedLoopController();
elevator2.configure(null, null, null);//set follower
    elevatorEncoder.setPosition(0);
    elevatorClosedLoopController.setReference(0, ControlType.kPosition);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Target", 0);
  }

}