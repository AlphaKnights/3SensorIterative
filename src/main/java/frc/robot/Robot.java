/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * * This is the code for the T-Shirt Cannon. Works pretty good.
 * 
 * Basic info: Drives with logitech flight stick, air compressor with button 12,
 * fire with triger, thumb button, and button 5/6 for top and bottom array.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private SpeedControllerGroup leftSideSpeedControllerGroup;
  private SpeedControllerGroup rightSideSpeedControllerGroup;
  private WPI_TalonSRX shooter = new WPI_TalonSRX(4);

  private static final int kJoystickChannel = 0;
  private static final int kGyroChannel = 0;
  private Joystick m_stick;
  private AnalogGyro m_gyro;

  public double integrative;
  public double previousError;
  public AHRS ahrs;
  public double rotation;
  public Ultrasonic m_ultSensor;

  public boolean aimNotComplete = true;
  public static int currentState = 1;

  public double Kp = 0.025f;
  public double I = 0.03;
  public double min_command = 0.19f;

  public edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry targetx = table.getEntry("tx");

  public double tx = targetx.getDouble(0);
  public double steering_adjust = 0.0f;
  public double error = tx;
  public double error_sign = error/Math.abs(error);

  @Override
  public void robotInit() {
    m_stick = new Joystick(kJoystickChannel);
    leftSideSpeedControllerGroup = new SpeedControllerGroup(new WPI_TalonSRX(0), new WPI_TalonSRX(3));
    rightSideSpeedControllerGroup = new SpeedControllerGroup(new WPI_TalonSRX(1), new WPI_TalonSRX(2));  
    m_myRobot = new DifferentialDrive(leftSideSpeedControllerGroup, rightSideSpeedControllerGroup);
    m_gyro = new AnalogGyro(kGyroChannel);
    ahrs = new AHRS(SerialPort.Port.kUSB);
    ahrs.resetDisplacement();
    ahrs.zeroYaw();
    m_ultSensor = new Ultrasonic(0,1,Unit.kMillimeters);
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    
    double[] movementList = adjustJoystickInput(m_stick.getY(), m_stick.getX(), m_stick.getThrottle());
    m_myRobot.arcadeDrive(movementList[0], movementList[1]);
    shooter.set(0);
    rotation = ahrs.getAngle();
    
   
    if (m_stick.getRawButton(4) == true) {
      shooter.set(-1);
      System.out.println(shooter.get());
    }
    else {
      shooter.set(0);
    }   
    
    //aims based on throttle, be careful not to kill the poor robot
    //if it doesn't work, blame the 20ms loop reset
    if (m_stick.getRawButton(10) == true) {
      aimNotComplete = true;
      while (aimNotComplete == true) {
        switch(currentState) {
          case 1 :
            autoAim();
          case 2 :
            adjust();
          case 3 :
            autoAim();
            aimNotComplete = false;
        }
      }
    }
  }  

  private double[] adjustJoystickInput(double yPower, double zPower, double Throttle) {
    double adjustedThrottle = Math.sqrt(((-1 * Throttle) + 1) / 2); // Seems like a pretty good throttle curve
    double yPowerOut = yPower * adjustedThrottle;
    double zPowerOut = zPower * adjustedThrottle * 0.85;

    double[] outputList = new double[] { yPowerOut, zPowerOut };
    return outputList;
  }

  private void autoAim() {
    //this code is assuming that positive error is clockwise, 
    //so if it goes in the wrong direction just multiply the final output by -1.

    //basically means that if limelight is looking at a target, and the target is 
    //seen to the right, it would have an error of +x, and steering_adjust would 
    //be positive, so the outermost wheels would spin in the same direction as error_sign.
    while (tx > 2.0 || tx < -2.0 ) {     //do you need to set these ==true?  doubt it.  an error could occur here
      if (error_sign != error/Math.abs(error)) { error = 0; }     
      integrative += (error * .02);  //time per loop  (reeeeeeeeeee)
      steering_adjust = error_sign * ((Kp * error + I*integrative) - min_command);
      m_myRobot.tankDrive(steering_adjust, -steering_adjust);     
    }
    if (tx < 2.0 && tx > -2.0) {
      m_myRobot.tankDrive(0, 0);
      currentState++;
    }    
  }

  private void adjust() {
    double displacement = m_ultSensor.getRangeMM() - (m_ultSensor.getRangeMM() * Math.cos(rotation));
    
    if (rotation/Math.abs(rotation) > 0) {
      while (rotation < 90) { //do you need to set these ==true? doubt it.  an error oculd occur here
        updateRotation();
        m_myRobot.tankDrive(m_stick.getThrottle(), -m_stick.getThrottle());
      }
    }
    else {
      while (rotation > -90) {
        updateRotation();
        m_myRobot.tankDrive(-m_stick.getThrottle(), m_stick.getThrottle());
      }
    }  

    while (m_ultSensor.getRangeMM() > displacement) {
      m_myRobot.tankDrive(m_stick.getThrottle(), m_stick.getThrottle());
    }

    if (rotation/Math.abs(rotation) > 0) {
      while (rotation > 0) {
        m_myRobot.tankDrive(-m_stick.getThrottle(), m_stick.getThrottle());
      }
    }
    else {
      while (rotation < 0) {
        m_myRobot.tankDrive(m_stick.getThrottle(), -m_stick.getThrottle());
      }
    }

    currentState++;
  }

  private void updateRotation() {
    while (rotation >= 360) {
      rotation = rotation - 360;
    }
    while (rotation < 0) {
      rotation = rotation + 360;
    }
  }
}