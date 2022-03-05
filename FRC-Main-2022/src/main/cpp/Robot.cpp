// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/TimedRobot.h>

  void Robot::RobotInit()
  {
    // Assigning Falcons
    m_ShooterLeft = new TalonFX(5);
    m_ShooterRight = new TalonFX(6);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.SetInverted(true);

    //ColorSensorV3

    AllianceColor = frc::DriverStation::GetAlliance();
    switch (AllianceColor)
    {
      case frc::DriverStation::Alliance::kBlue:
           frc::SmartDashboard::PutString("AllianceColor", "Blue");
           break;
      case frc::DriverStation::Alliance::kRed:
           frc::SmartDashboard::PutString("AllianceColor", "Red");
           break;
      case frc::DriverStation::Alliance::kInvalid:
           frc::SmartDashboard::PutString("AllianceColor", "?");
           break;
      
    }

  //Limelight code

    table->PutNumber("ledMode", 3);
    table->PutNumber("camMode", 0);

  //Grip code

    gripStartPosition = m_climberEncoder.GetPosition();
    gripState = gripStopped;
    gripMax = gripStartPosition + 0.5;

  //Turret code

    m_turretEncoder.SetPosition(0);
    homingState = manual;
    turretMax = -13;

  }

  void Robot::RobotPeriodic()
  {

    //Gyro

    frc::SmartDashboard::PutNumber("YawAngle", m_imu.GetAngle().value());
    frc::SmartDashboard::PutNumber("XCompAngle", m_imu.GetXComplementaryAngle().value());
    frc::SmartDashboard::PutNumber("YCompAngle", m_imu.GetYComplementaryAngle().value());
    m_setDecRate = frc::SmartDashboard::GetBoolean("SetDecRate", false);
    m_decRate = frc::SmartDashboard::GetNumber("DecRate", 4);
    m_runCal = frc::SmartDashboard::GetBoolean("RunCal", false);
    m_configCal = frc::SmartDashboard::GetBoolean("ConfigCal", false);
    m_reset = frc::SmartDashboard::GetBoolean("Reset", false);
    m_setYawAxis = frc::SmartDashboard::GetBoolean("SetYawAxis", false);
    m_yawSelected = m_yawChooser.GetSelected();
    
    //ColorSensorV3

    frc::Color detectedColor = m_colorSensor.GetColor();
    double IR = m_colorSensor.GetIR();
    uint32_t proximity = m_colorSensor.GetProximity();

    frc::SmartDashboard::PutNumber("ColorSensor.Color...Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("ColorSensor.Color..Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("ColorSensor.Color.Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("ColorSensor.IR", IR);
    frc::SmartDashboard::PutNumber("ColorSensor.Proximity", proximity);

    //Encoders

    frc::SmartDashboard::PutNumber("Grip Encoder Position", m_gripEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Turret Encoder Position", m_turretEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climber Encoder Position", m_climberEncoder.GetPosition());

    //Limit switch

    frc::SmartDashboard::PutBoolean("Turret Limit Switch Position", m_turretlimitSwitch.Get());

    //Shooter

    shooterRightOutput = m_ShooterRight->GetSelectedSensorVelocity();

    frc::SmartDashboard::PutNumber("Right Shooter Velocity", shooterRightOutput);

    //Homing mode

    switch (homingState)
    {
    case automatic:
      frc::SmartDashboard::PutString("Homing State", "Automatic");
        break;
    case manual:
      frc::SmartDashboard::PutString("Homing State", "Manual");
        break;
    case homingOff:
      frc::SmartDashboard::PutString("Homing State", "Off");
        break;
    default:
      break;
    }

        switch (gripState)
    {
    case gripOpening:
      frc::SmartDashboard::PutString("Grip State", "Opening");
        break;
    case gripClosing:
      frc::SmartDashboard::PutString("Grip State", "Closing");
        break;
    case gripStopped:
      frc::SmartDashboard::PutString("Grip State", "Stopped");
        break;
    default:
      break;
    }

    //Shooter

    /*
    frc::SmartDashboard::PutNumber("Left Shooter Output", shooterLeftOutput);
    frc::SmartDashboard::PutNumber("Right Shooter Output", shooterRightOutput);
    */
  }

  void Robot::AutonomousInit(){
    autoHoming = false;

    frontRightMax = 42;
    rearLeftMax = 42 * -1; 

    m_frontRightEncoder.SetPosition(0);
    m_rearLeftEncoder.SetPosition(0);

    m_time.Reset();

    autoDriven = false;

  }

  void Robot::AutonomousPeriodic(){
        
    /*
    if(autoHoming == true){
    
      if(m_turretlimitSwitch.Get()){
        m_turretMotor.Set(0);
        autoHoming = false;
        turretEncoderStart = m_turretEncoder.GetPosition();
      }
      else{
        m_turretMotor.Set(0.1);
      }
    }
  */
      m_ShooterRight->Set(ControlMode::PercentOutput, shooterTargetSpeed);
      m_ShooterLeft->Set(ControlMode::PercentOutput, shooterTargetSpeed * -1);
    
    /*
    (m_ShooterRight->GetSelectedSensorVelocity() >= ){
      m_ShooterRight->Set(ControlMode::PercentOutput, shooterTargetSpeed);
      m_ShooterLeft->Set(ControlMode::PercentOutput, shooterTargetSpeed * -1);
    }
    */
    
    //if()

    shooterLeftOutput = m_ShooterLeft->GetMotorOutputPercent();
    shooterRightOutput = m_ShooterRight->GetMotorOutputPercent();

    if(m_ShooterRight->GetSelectedSensorVelocity() >= 15000){
      m_intake.Set(intakeTargetSpeed);
    }

    if(m_frontRightEncoder.GetPosition() < frontRightMax && m_rearLeftEncoder.GetPosition() > rearLeftMax){
      m_leftMotor.Set(0.1);
      m_rightMotor.Set(0.1);
      autoDriven = true;
    }
    else{
      m_leftMotor.Set(0);
      m_rightMotor.Set(0);

      if(autoDriven == true){
      m_time.Start();
      }
    }
    //frc::SmartDashboard::PutNumber("Left Shooter Output", shooterLeftOutput);
    //frc::SmartDashboard::PutNumber("Right Shooter Output", shooterRightOutput);

    if(m_time.Get().value() >= 3){
    
      if(m_turretlimitSwitch.Get()){
        m_turretMotor.Set(0);
        //autoHoming = false;
        m_turretEncoder.SetPosition(0);     
        homingState = homingOff;
        }
      else{
        m_turretMotor.Set(0.1);
        m_ShooterRight->Set(ControlMode::PercentOutput, 0);
        m_ShooterLeft->Set(ControlMode::PercentOutput, 0);
        m_intake.Set(0);
      }
    }
  }

  void Robot::TeleopInit(){
  
  }
  
  void Robot::TeleopPeriodic()
  {

    //Turret Code

    double TriggerSpeed = 0.1;
    double LeftTrigger = buttonBoard.GetRawButton(4);
    double RightTrigger = buttonBoard.GetRawButton(8);

    double TurretSpeed = -1 * TriggerSpeed * LeftTrigger + RightTrigger * TriggerSpeed;

    turretPosition = m_turretEncoder.GetPosition();

    switch (homingState)
    {
    case automatic:
      m_turretMotor.Set(TriggerSpeed);
      break;
    case manual:
      if(TurretSpeed > 0){
        m_turretMotor.Set(TurretSpeed);
      }
      else{
        m_turretMotor.Set(0);
      }
      break;
    case homingOff:
      if(TurretSpeed < 0 && turretPosition > turretMax){
        m_turretMotor.Set(TurretSpeed);
      } 
      else if(TurretSpeed > 0 && turretPosition < 0){
        m_turretMotor.Set(TurretSpeed);
      }
      else{
        m_turretMotor.Set(0);
      }
      break;
    default:
      break;
    }

    m_robotDrive.TankDrive(-JLeft.GetY()*0.85,-JRight.GetY()*0.85);

    //Shooter & intake Code 

      /*
      frc::SmartDashboard::PutBoolean("AButtonPress", m_driverController.GetAButtonPressed());
      frc::SmartDashboard::PutBoolean("AButtonRelease", m_driverController.GetAButtonReleased());
      frc::SmartDashboard::PutBoolean("AButton", m_driverController.GetAButton());
      frc::SmartDashboard::PutBoolean("XButton", m_driverController.GetXButton());
      */

      if(buttonBoard.GetRawButton(11)){
        m_ShooterLeft->Set(ControlMode::PercentOutput, 0);
        m_ShooterRight->Set(ControlMode::PercentOutput, 0);
        m_intake.Set(0.0); 
      }
      else if(buttonBoard.GetRawButton(9)){
        m_ShooterLeft->Set(ControlMode::PercentOutput, shooterTargetSpeed * -1);
        m_ShooterRight->Set(ControlMode::PercentOutput, shooterTargetSpeed);
      }
      if(buttonBoard.GetRawButton(3)){
        m_ShooterLeft->Set(ControlMode::PercentOutput, shooterSlowSpeed * -1);
        m_ShooterRight->Set(ControlMode::PercentOutput, shooterSlowSpeed);

      }
      if(buttonBoard.GetRawButtonPressed(10)){
        m_intake.Set(intakeTargetSpeed);
      }
      else if(buttonBoard.GetRawButtonPressed(7)){
        m_intake.Set(intakeTargetSpeed * -1);
      }
      //shooterLeftOutput = m_ShooterLeft->GetMotorOutputPercent();
      //shooterRightOutput = m_ShooterRight->GetMotorOutputPercent();

    // Climber winch code

      if(buttonBoard.GetRawButton(1)){
        m_climberWinch.Set(-0.5);
      }
      else if(buttonBoard.GetRawButton(5)){
        m_climberWinch.Set(0.5);
      }
      else{
        m_climberWinch.Set(0.0);
      }
      
    
    // Climber grip code

      //grip open is 

      if(buttonBoard.GetRawButtonPressed(6)){
        gripState = gripOpening;
      };
      if(buttonBoard.GetRawButtonPressed(2)){
        gripState = gripClosing;
      }

      gripPosition = m_gripEncoder.GetPosition();

      if(gripState == gripClosing && gripPosition <= gripMax){
        m_climberGrip.Set(0.15);
      }
      else if(gripState == gripOpening && gripPosition >= gripStartPosition){
        m_climberGrip.Set(-0.15);
      }
      else{
        gripState = gripStopped;
        m_climberGrip.Set(0.0);
      }
      

    //ColorSensorV3 Code
    
    //Homing code

    if(homingState == manual || homingState == automatic){

      if(m_turretlimitSwitch.Get()){
        m_turretEncoder.SetPosition(0);
        homingState = homingOff;
      }
    } 
  }

//Destructor (Cleans up stuff)

Robot::~Robot(){
 delete m_ShooterLeft;
 delete m_ShooterRight;
}


#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
