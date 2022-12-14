// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <iostream>
#include <frc/Timer.h>
#include <string>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <chrono>
#include <thread>
#include <math.h>
#include <cameraserver/CameraServer.h>
#include "AHRS.h"



class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;


  void drive(double FWD, double STR, double RCW){
    yaw = ahrs->GetYaw();
    double rrsa = m_rrsensor.GetPosition();
    //yaw = ahrs->GetAngle();
    if(yaw<0){
      yaw *= -1;
      yaw = (360 - yaw); 
    }
    //std::cout<<yaw<<std::endl;
    yaw = (yaw * (M_PI/180));

    double temp = ((FWD * cos(yaw)) + STR * sin(yaw));
    STR = ((-FWD) * sin(yaw)) + (STR * cos(yaw));
    FWD = temp;
    //RCW *= 10;
    //std::cout<<"FWD: "<<FWD<<" STR: "<< STR <<" ANGLE: "<<yaw<<std::endl;

    L = 25.75; //wheelbase (from center of front wheel to center of back wheel) unit doesn't matter because these will only be used as ratios but works well with meters
	  W = 25.75; //wheelbase (from center of left wheel to center of right wheel) unit doesn't matter because these will only be used as ratios but works well with meters
	  R = sqrt((L * L) + (W * W));

	  A = STR - RCW * (L / R);
	  B = STR + RCW * (L / R);
	  C = FWD - RCW * (W / R);
	  D = FWD + RCW * (W / R);

	  ws1 = sqrt((B * B) + (C * C));
	  wa1 = atan2(B, C) * 180 / M_PI;
	  ws2 = sqrt((B * B) + (D * D));
	  wa2 = atan2(B, D) * 180 / M_PI;
	  ws3 = sqrt((A * A) + (D * D));
	  wa3 = atan2(A, D) * 180 / M_PI;
	  ws4 = sqrt((A * A) + (C * C));
	  wa4 = atan2(A, C) * 180 / M_PI;

    /*if ((abs(wa1 - oldwa1) > 90) && (abs(wa1 - oldwa1) < 270)) {
      if(oldwa1<0){
        oldwa1 *= -1;
        oldwa1 = (360 - oldwa1);
      }
      wa1 = (wa1 + 180.0);
      wa1 = fmod(wa1, 360);
      ws1 *= -1;
    }

    if ((abs(wa2 - oldwa2) > 90) && (abs(wa2 - oldwa2) < 270)) {
      if(oldwa2<0){
        oldwa2 *= -1;
        oldwa2 = (360 - oldwa2);
      }
      wa2 = (wa2 + 180.0);
      wa2 = fmod(wa2, 360);
      ws2 *= -1;
    }

    if ((abs(wa3 - oldwa3) > 90) && (abs(wa3 - oldwa3) < 270)) {
      if(oldwa3<0){
        oldwa3 *= -1;
        oldwa3 = (360 - oldwa3);
      }
      wa3 = (wa3 + 180.0);
      wa3 = fmod(wa3, 360);
      ws3 *= -1;
    }

    if ((abs(wa4 - oldwa4) > 90) && (abs(wa4 - oldwa4) < 270)) {
      if(oldwa4<0){
        oldwa4 *= -1;
        oldwa4 = (360 - oldwa4);
      }
      wa4 = (wa4 + 180.0);
      wa4 = fmod(wa4, 360);
      ws4 *= -1;
    }*/

	  wa1 *= conversion;
	  wa2 *= conversion;
	  wa3 *= conversion;
	  wa4 *= conversion;

	  double max = ws1;
	  if (ws2 > max)max = ws2;
	  if (ws3 > max)max = ws3;
	  if (ws4 > max)max = ws4;
	  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }


    m_rra.Set(TalonFXControlMode::Position, wa4);
    m_fra.Set(TalonFXControlMode::Position, wa1);
    m_rla.Set(TalonFXControlMode::Position, wa3);
    m_fla.Set(TalonFXControlMode::Position, wa2);

    std::cout<<"cancoder: "<<rrsa<<" angle: "<<wa4<<std::endl;

    oldwa1 = wa1;
    oldwa2 = wa2;
    oldwa3 = wa3;
    oldwa4 = wa4;

    double PercentOut = 0.2;

    m_rrd.Set(PercentOut*ws4);
    m_frd.Set(PercentOut*ws1);
    m_rld.Set(PercentOut*ws3);
    m_fld.Set(PercentOut*ws2);
  }


  private:
    //****************************************************CONTROLLERS****************************************************
    frc::XboxController m_driverController{0};

    //****************************************************MOTORS AND ENCODERS****************************************************
    WPI_TalonFX m_fra{9};
    WPI_TalonFX m_frd{11};

    WPI_TalonFX m_rra{12};
    WPI_TalonFX m_rrd{10};

    WPI_TalonFX m_fla{0};
    WPI_TalonFX m_fld{5};

    WPI_TalonFX m_rla{15};
    WPI_TalonFX m_rld{18};


    WPI_CANCoder m_rrsensor{5};
    WPI_CANCoder m_frsensor{2};
    WPI_CANCoder m_rlsensor{1};
    WPI_CANCoder m_flsensor{3};

    //****************************************************SWERVE DRIVE VARIABLES****************************************************
    double conversion = 4096.0/ 360.0;

    double L;
    double W;
    double R;

	  double A;
    double B;
    double C;
    double D;

    double ws1;
    double wa1;
    double ws2;
    double wa2;
    double ws3;
    double wa3;
    double ws4;
    double wa4;

    double oldwa1;
    double oldwa2;
    double oldwa3;
    double oldwa4;

    //****************************************************NAVX-MXP****************************************************
    AHRS *ahrs;
    double yaw;
  };
