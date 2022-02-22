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

    //Encoders

    frc::SmartDashboard::PutNumber("Encoder Position", m_gripEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Encoder Position", m_turretEncoder.GetPosition());

  }

  void Robot::TeleopInit(){
  
  }
  
  void Robot::TeleopPeriodic()
  {

    //Turret Code

    double TriggerSpeed = 0.05;
    double LeftTrigger = buttonBoard.GetRawButton(4);
    double RightTrigger = buttonBoard.GetRawButton(8);

    double TurretSpeed = -1 * TriggerSpeed * LeftTrigger + RightTrigger * TriggerSpeed;

    m_turretMotor.Set(TurretSpeed);

    m_robotDrive.TankDrive(-JLeft.GetY()*0.85,-JRight.GetY()*0.85);

    //Shooter & intake Code 

      /*
      frc::SmartDashboard::PutBoolean("AButtonPress", m_driverController.GetAButtonPressed());
      frc::SmartDashboard::PutBoolean("AButtonRelease", m_driverController.GetAButtonReleased());
      frc::SmartDashboard::PutBoolean("AButton", m_driverController.GetAButton());
      frc::SmartDashboard::PutBoolean("XButton", m_driverController.GetXButton());
      */

      if(buttonBoard.GetRawButton(10)){
        m_ShooterLeft->Set(ControlMode::PercentOutput, 0);
        m_ShooterRight->Set(ControlMode::PercentOutput, 0);
        m_intake.Set(0.0); 
      }
      else if(buttonBoard.GetRawButton(9)){
        m_ShooterLeft->Set(ControlMode::PercentOutput, shooterTargetSpeed); 
        m_ShooterRight->Set(ControlMode::PercentOutput, shooterTargetSpeed * -1);
        m_intake.Set(-0.65);
        }

    /* Intake code

      if(buttonBoard.GetRawButton(9)){
        m_intake.Set(0.05);
      }
      else if(buttonBoard.GetRawButton(11)){
        m_intake.Set(0.05 * -1);
      }
      else if(buttonBoard.GetRawButton(10)){
        m_intake.Set(0.0);
      }
    */

    // Climber winch code

      if(buttonBoard.GetRawButton(1)){
        m_climberWinch.Set(0.5);
      }
      else if(buttonBoard.GetRawButton(5)){
        m_climberWinch.Set(-0.5);
      }
      else{
        m_climberWinch.Set(0.0);
      }
      
    
    // Climber grip code
      
      if(buttonBoard.GetRawButton(2)){
        m_climberGrip.Set(-0.05);
      }
      else if(buttonBoard.GetRawButton(6)){
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
