// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/TimedRobot.h>

  void Robot::RobotInit()
  {
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

    //Code that sends videoioutput from a webcam to the driver station

    #if defined(__linux__) || defined(_WIN32)
    frc::CameraServer::StartAutomaticCapture();
  #else
    std::fputs("Vision only available on Linux or Windows.\n", stderr);
    std::fflush(stderr);
  #endif
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
    //frc::SmartDashboard
  }

  void Robot::TeleopInit(){
  
  }
  
  void Robot::TeleopPeriodic()
  {

    //Turret Code

    double BumperSpeed = 0.05;
    bool LeftBumper = m_driverController.GetLeftBumper();
    bool RightBumper = m_driverController.GetRightBumper();

    double TurretSpeed = -1 * BumperSpeed * LeftBumper + RightBumper * BumperSpeed;

    m_robotDrive.TankDrive(-m_driverController.GetLeftY()*0.85,-m_driverController.GetRightY()*0.85);

    //m_turretMotor.Set(TurretSpeed);

    //m_ShooterLeft.(.05);
    //m_ShooterLeft.Set(ControlMode::Velocity, shooterTargetSpeed);



    //frc::SmartDashboard::PutNumber("m_turretMotor",LeftBumper);

    //ColorSensorV3 Code

  }

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
