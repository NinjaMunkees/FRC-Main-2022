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

    //m_driverController.GetAButtonPressed();
  }

  void Robot::TeleopPeriodic()
  {
    m_robotDrive.TankDrive(-m_driverController.GetLeftY(),
                           -m_driverController.GetRightY());
  }

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
