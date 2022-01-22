// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/ADIS16470_IMU.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    m_frontRight.SetInverted(true);
    m_rearRight.SetInverted(true);

  m_autoChooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_autoChooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_yawChooser.SetDefaultOption(kYawDefault, kYawDefault);
  m_yawChooser.AddOption(kYawXAxis, kYawXAxis);
  m_yawChooser.AddOption(kYawYAxis, kYawYAxis);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
  frc::SmartDashboard::PutData("IMUYawAxis", &m_yawChooser);
  frc::SmartDashboard::PutBoolean("RunCal", false);
  frc::SmartDashboard::PutBoolean("ConfigCal", false);
  frc::SmartDashboard::PutBoolean("Reset", false);
  frc::SmartDashboard::PutBoolean("SetYawAxis", false);
  frc::SmartDashboard::PutBoolean("SetDecRate", false);
  frc::SmartDashboard::PutNumber("DecRate", m_decRate);
  }

  void TeleopPeriodic() override
  {
    /* Use the joystick X axis for lateral movement, Y axis for forward
     * movement, and Z axis for rotation.
     */
    m_robotDrive.DriveCartesian(m_stick.GetX(), m_stick.GetY(), m_stick.GetZ());
  }

private:
  static constexpr int kFrontLeftChannel = 0;
  static constexpr int kRearLeftChannel = 1;
  static constexpr int kFrontRightChannel = 2;
  static constexpr int kRearRightChannel = 3;

  static constexpr int kJoystickChannel = 0;

  frc::PWMSparkMax m_frontLeft{kFrontLeftChannel};
  frc::PWMSparkMax m_rearLeft{kRearLeftChannel};
  frc::PWMSparkMax m_frontRight{kFrontRightChannel};
  frc::PWMSparkMax m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};

  frc::Joystick m_stick{kJoystickChannel};
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
