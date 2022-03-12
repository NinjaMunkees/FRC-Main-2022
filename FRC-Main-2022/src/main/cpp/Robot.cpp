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

    detectedBallColor = InvalidBall;

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

  //Turret code

    m_turretEncoder.SetPosition(0);
    homingState = manual;
    turretMax = -13;
    homingDone = false;

  }

  void Robot::RobotPeriodic()
  {

    //Gyro

    //frc::SmartDashboard::PutNumber("YawAngle", m_imu.GetAngle().value());
    //frc::SmartDashboard::PutNumber("XCompAngle", m_imu.GetXComplementaryAngle().value());
    //frc::SmartDashboard::PutNumber("YCompAngle", m_imu.GetYComplementaryAngle().value());
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

    if(detectedColor.red >= 0.269){
      detectedBallColor = RedBall;
    }
    else if(detectedColor.blue >= 0.32){
      detectedBallColor = BlueBall;
    }
    else{
      detectedBallColor = InvalidBall;
    }
    
    switch (detectedBallColor)
    {
      case BlueBall:
           frc::SmartDashboard::PutString("Detected Ball Color", "Blue");
           break;
      case RedBall:
           frc::SmartDashboard::PutString("Detected Ball Color", "Red");
           break;
      case InvalidBall:
           frc::SmartDashboard::PutString("Detected Ball Color", "?");
           break;
      
    }

    //Encoders

    frc::SmartDashboard::PutNumber("Grip Encoder Position", m_gripEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Turret Encoder Position", m_turretEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climber Encoder Position", m_climberEncoder.GetPosition());

    //Limit switch

    frc::SmartDashboard::PutBoolean("Turret Limit Switch Position", m_turretlimitSwitch.Get());

    //Shooter

    shooterRightOutput = m_ShooterRight->GetSelectedSensorVelocity();

    frc::SmartDashboard::PutNumber("Right Shooter Velocity", shooterRightOutput);

    frc::SmartDashboard::PutNumber("Shooter High Speed", shooterFastSpeed);

    //Homing mode

    JLeftZ = JLeft.GetZ() * 15;

    frc::SmartDashboard::PutNumber("JLeft Z", JLeftZ);

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
  }

  void Robot::AutonomousInit(){

    frontRightMax = 42;
    rearLeftMax = 42 * -1; 

    m_frontRightEncoder.SetPosition(0);
    m_rearLeftEncoder.SetPosition(0);

    m_time.Reset();
    m_timeToo.Reset();

    autoDriven = false;

  }

  void Robot::AutonomousPeriodic(){

    m_timeToo.Start();
        
    m_ShooterRight->Set(ControlMode::PercentOutput, shooterTargetSpeed);
    m_ShooterLeft->Set(ControlMode::PercentOutput, shooterTargetSpeed * -1);

    shooterLeftOutput = m_ShooterLeft->GetMotorOutputPercent();
    shooterRightOutput = m_ShooterRight->GetMotorOutputPercent();

    if(m_timeToo.Get().value() >= 2.75){
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

    if(m_time.Get().value() >= 4){
    
      if(m_turretlimitSwitch.Get()){
        m_turretMotor.Set(0);
        homingDone = true;
        m_turretEncoder.SetPosition(0);     
        homingState = homingOff;
      }
      else{
        m_turretMotor.Set(0.1);
        m_intake.Set(0);
      }
        m_ShooterRight->Set(ControlMode::PercentOutput, 0);
        m_ShooterLeft->Set(ControlMode::PercentOutput, 0);
        m_intake.Set(0);
    }
  }

  void Robot::TeleopInit(){
    m_time.Stop();
    m_time.Reset();

    //Grip code

    gripStartPosition = 0;
    m_gripEncoder.SetPosition(0);
    gripState = gripStopped;
    gripMax = gripStartPosition + 1.3;
  }
  
  void Robot::TeleopPeriodic()
  {

    //shooterFastSpeed = 0.6;  //(JLeft.GetZ() + 1) * 0.25 + 0.5;

    //Turret Code

    double LeftTrigger = buttonBoard.GetRawButton(4);
    double RightTrigger = buttonBoard.GetRawButton(8);

    double TurretSpeed = -1 * TriggerSpeed * LeftTrigger + RightTrigger * TriggerSpeed;

    turretPosition = m_turretEncoder.GetPosition();

    if(homingDone == true && buttonBoard.GetRawButtonPressed(3)){
      homingState = automatic;
    }
    
    if(buttonBoard.GetRawButtonPressed(4) || buttonBoard.GetRawButtonPressed(8) || buttonBoard.GetRawButtonPressed(11)){
      if(homingDone){
        homingState = homingOff;
      }
      else{
        homingState = manual;
      }
    }

    switch (homingState)
    {
    case automatic:
      targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
      targetDetect = table->GetBoolean("tv",false);

      turretTargetSpeed = TriggerSpeed * ((targetOffsetAngle_Horizontal + JLeftZ /*+ 8 - m_turretEncoder.GetPosition() * 12 / turretMax*/) / 20.0);
    
      if(turretTargetSpeed < 0 && m_turretEncoder.GetPosition() > turretMax){
        m_turretMotor.Set(turretTargetSpeed);
      }
      else if(m_turretEncoder.GetPosition() < 0 && turretTargetSpeed > 0){
        m_turretMotor.Set(turretTargetSpeed);
      }
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

      if(buttonBoard.GetRawButton(11)){
        shooterTargetSpeed = 0;
        m_intake.Set(0.0); 
        shooterAlive = false;
      }
      else if(JLeft.GetRawButton(8)){
        shooterTargetSpeed = shooterSlowSpeed;
      }
      else if(JLeft.GetRawButton(9)){
        shooterTargetSpeed = shooterFastSpeed;
      }
      else if(buttonBoard.GetRawButton(9)){
        shooterTargetSpeed = shooterMidSpeed;
        shooterAlive = true;
      }
      else{
        if(shooterAlive == true){
          shooterTargetSpeed = shooterMidSpeed;
        }
        else{
          shooterTargetSpeed = 0;
        }
      }

      if(buttonBoard.GetRawButtonPressed(10)){
        m_intake.Set(intakeTargetSpeed);
      }
      else if(buttonBoard.GetRawButtonPressed(7)){
        m_intake.Set(intakeTargetSpeed * -1);
      }

      //The below color sensor code over-rides user input for shooter speed

      /*
      if(detectedBallColor == RedBall && AllianceColor == frc::DriverStation::Alliance::kBlue){
        shooterTargetSpeed = 0;
        m_time.Reset();
        m_time.Start();
      }
      else if(detectedBallColor == BlueBall && AllianceColor == frc::DriverStation::Alliance::kRed){
        shooterTargetSpeed = 0;
        m_time.Reset();
        m_time.Start();
      }
      if(m_time.Get().value() >= 1.5 && m_time.Get().value() < 1.6){
        m_time.Stop();
        m_time.Reset();
        shooterTargetSpeed = shooterFastSpeed;
      }
      */

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
      gripSpeed = 0.1;

      if(gripState == gripClosing && gripPosition < gripMax){
        m_climberGrip.Set(((gripMax - gripPosition) / gripMax) * gripSpeed + 0.05);
      }
      else if(gripState == gripOpening && gripPosition > 0){
        m_climberGrip.Set(((gripStartPosition - gripPosition) / gripMax) * gripSpeed - 0.05);
      }
      else{
        gripState = gripStopped;
        m_climberGrip.Set(0.0);
      }
      
    //Homing code

    if(homingState == manual){

      if(m_turretlimitSwitch.Get()){
        m_turretEncoder.SetPosition(0);
        homingState = homingOff;
        homingDone = true;
      }
    }
    m_ShooterLeft->Set(ControlMode::PercentOutput, shooterTargetSpeed * -1);
    m_ShooterRight->Set(ControlMode::PercentOutput, shooterTargetSpeed);
  }

  void Robot::TestPeriodic(){
    
    double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    bool targetDetect = table->GetBoolean("tv",false);

    double visionSpeed = TriggerSpeed * (targetOffsetAngle_Horizontal / 30.0);

    frc::SmartDashboard::PutNumber("Vision Horizontal Offset", targetOffsetAngle_Horizontal);
    frc::SmartDashboard::PutNumber("Turret Speed for Vision", visionSpeed);

    m_turretMotor.Set(visionSpeed);

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
