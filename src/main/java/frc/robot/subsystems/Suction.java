// // Copyright (c) 2023 FRC Team 2881 - The Lady Cans
// //
// // Open Source Software; you can modify and/or share it under the terms of BSD
// // license file in the root directory of this project.

// package frc.robot.subsystems;


// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.PneumaticHub;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Suction extends SubsystemBase {
  
//   private final PneumaticHub m_pneumaticHub;
//   private final CANSparkMax m_motor1;
//   //private final CANSparkMax m_motor2;
//   private final Solenoid m_solenoid1;
//   //private final Solenoid m_solenoid2;
//   private boolean m_isEnabled = false;
//   private boolean m_hasVacuum = false;
  

//   public Suction() {
//     m_pneumaticHub = new PneumaticHub();
//     m_motor1 = new CANSparkMax(Constants.Suction.kMotor1Id, MotorType.kBrushless);
//     m_motor1.restoreFactoryDefaults();
//     m_motor1.setInverted(false);
//     m_motor1.setIdleMode(IdleMode.kBrake);
//     m_motor1.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

//     //TODO We eventually will have two solenoids and two motors for the mechanism 
//     //m_motor2 = new CANSparkMax(Constants.Suction.kMotorId, MotorType.kBrushless);
//     //m_motor2.restoreFactoryDefaults();
//     //m_motor2.setInverted(false);
//     //m_motor2.setIdleMode(IdleMode.kBrake);
//     //m_motor2.setSmartCurrentLimit(Constants.Suction.kCurrentLimit);

//     m_solenoid1 = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoid1Id);
//     //m_solenoid2 = new Solenoid(PneumaticsModuleType.REVPH, Constants.Suction.kSolenoidId2);

//   }

//   @Override
//   public void periodic() {
//     double pressure1 = m_pneumaticHub.getPressure(Constants.Suction.kPressureSensor1Id);
//     //double pressure2 = m_pneumaticHub.getPressure(Constants.Suction.kPressureSensor2Id);
//     double pressure = pressure1;

//     //Automated reenabling of the suction system
//     if (m_isEnabled) {
//       m_solenoid1.set(false);
//       //m_solenoid2.set(false);
//       if (!m_hasVacuum) {
//         if (pressure > Constants.Suction.kTargetPressure) {
//           m_motor1.set(Constants.Suction.kMaxSpeed);
//           //m_motor2.set(Constants.Suction.kMaxSpeed);
//         } else {
//           m_motor1.set(0);
//           //m_motor2.set(0);
//           m_hasVacuum = true;
//         }
//       } else {
//         if (pressure > Constants.Suction.kMinimumPressure) {
//           m_hasVacuum = false;
//         }
//       }
//     } else {
//       m_motor1.set(0);
//       //m_motor2.set(0);
//       m_solenoid1.set(true);
//       //m_solenoid2.set(true);
//       m_hasVacuum = false;
//     }

//     SmartDashboard.putNumber("Suction/Pressure/0", pressure);
//     SmartDashboard.putNumber("Suction/Pressure/1", pressure1);
//     //SmartDashboard.putNumber("Suction/Pressure/2", pressure2);
//   }

//   public void enable() {  
//     m_isEnabled = true;
//   }

//   public void disable() {
//     m_isEnabled = false;
//   }
// }
