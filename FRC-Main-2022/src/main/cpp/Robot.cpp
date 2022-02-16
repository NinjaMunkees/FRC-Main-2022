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
    //m_ShooterLeft->SetInverted(true);

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

    //Code that sends videoioutput from a webcam to the driver station

    /*

    #if defined(__linux__) || defined(_WIN32)
    frc::CameraServer::StartAutomaticCapture();
  #else
    std::fputs("Vision only available on Linux or Windows.\n", stderr);
    std::fflush(stderr);
  #endif
  
  */
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

  }

  void Robot::TeleopInit(){
  
  }
  
  void Robot::TeleopPeriodic()
  {

    //Turret Code

    double TriggerSpeed = 0.05;
    double LeftTrigger = m_driverController.GetLeftTriggerAxis();
    double RightTrigger = m_driverController.GetRightTriggerAxis();

    double TurretSpeed = -1 * TriggerSpeed * LeftTrigger + RightTrigger * TriggerSpeed;

     m_turretMotor.Set(TurretSpeed);

    m_robotDrive.TankDrive(-m_driverController.GetLeftY()*0.85,-m_driverController.GetRightY()*0.85);

    //Shooter Code 

      frc::SmartDashboard::PutBoolean("AButtonPress", m_driverController.GetAButtonPressed());
      frc::SmartDashboard::PutBoolean("AButtonRelease", m_driverController.GetAButtonReleased());
      frc::SmartDashboard::PutBoolean("AButton", m_driverController.GetAButton());
      frc::SmartDashboard::PutBoolean("XButton", m_driverController.GetXButton());

      if(m_driverController.GetAButton()){
        m_ShooterLeft->Set(ControlMode::Velocity, shooterTargetSpeed); 
        m_ShooterRight->Set(ControlMode::Velocity, shooterTargetSpeed * -1);
      }
      else {  
        m_ShooterLeft->Set(ControlMode::PercentOutput, 0);
        m_ShooterRight->Set(ControlMode::PercentOutput, 0); 
      }

    // Intake code

      if(m_driverController.GetYButtonPressed()){
        m_intake.Set(0.05);
      }
      else if(m_driverController.GetYButtonPressed() && m_driverController.GetLeftBumper()){
        m_intake.Set(0.05 * -1);
      }
      else if(m_driverController.GetYButton() && m_driverController.GetRightBumper()){
        m_intake.Set(0.0);
      }
      
    // Climber winch code

      if(m_driverController.GetBButton() && m_driverController.GetLeftBumper()){
        m_climberWinch.Set(0.5);
      }
      else if(m_driverController.GetBButton() && m_driverController.GetRightBumper()){
        m_climberWinch.Set(-0.5);
      }
      else{
        m_climberWinch.Set(0.0);
      }
    
    // Climber grip code
      
      if(m_driverController.GetXButton() && m_driverController.GetLeftBumper()){
        m_climberGrip.Set(-0.05);
      }
      else if(m_driverController.GetXButton() && m_driverController.GetRightBumper()){
        m_climberGrip.Set(0.05);
      }
      else{
        m_climberGrip.Set(0.0);
      }
      

    //ColorSensorV3 Code

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
