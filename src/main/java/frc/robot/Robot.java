/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.joystick.*;
import frc.robot.loops.*;
import frc.robot.subsystems.Drive;
import frc.robot.command_status.*;
import frc.robot.Constants;

/*
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  ColorSorter colorSorter;
  Intake intake; 
  Outtake goodOuttake;
  Outtake badOuttake;
  Hopper hopper;
  Drive drive = Drive.getInstance();
	LoopController loopController;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.addDefault("Default Auto", kDefaultAuto);
    m_chooser.addObject("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    colorSorter = ColorSorter.getInstance();
    intake = Intake.getInstance();
    goodOuttake = Outtake.getGoodInstance();
    badOuttake = Outtake.getBadInstance();
    hopper = Hopper.getInstance();
    loopController = new LoopController();
    loopController.register(drive.getVelocityPIDLoop());
    loopController.register(DriveLoop.getInstance());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    // start ball sorter
    //start intake action
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    colorSorter.run();
    intake.run();
    loopController.start();
    JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
    hopper.run(controls.getButton(Constants.kXboxButtonX));
    goodOuttake.run(controls.getButton(Constants.kXboxButtonA));
    badOuttake.run(controls.getButton(Constants.kXboxButtonB));
    // System.out.printf("BtnA: %b, BtnB: %b\n", controls.getButton(Constants.kXboxButtonA), controls.getButton(Constants.kXboxButtonB));
    DriveCommand driveCmd = controls.getDriveCommand();
    drive.setOpenLoop(driveCmd);
  } 


  @Override
  public void testPeriodic() {
  }
}
