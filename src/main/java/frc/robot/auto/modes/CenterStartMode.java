package frc.robot.auto.modes;

import frc.robot.Constants;
import frc.robot.auto.actions.*;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveStraightAction;

import edu.wpi.first.wpilibj.Talon;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class CenterStartMode extends AutoModeBase {

    public CenterStartMode() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Center Start Mode");
        PathSegment.Options pathOptions	= new PathSegment.Options(Constants.kPathFollowingMaxVel, SlowerAccel, 48, false);
        PathSegment.Options tightTurnOptions	= new PathSegment.Options(Constants.kPathFollowingMaxVel/2, SlowerAccel, 24, false);
        //Vector2d backupPosition = 		new Vector2d(24, 0);
		//Vector2d cubePickupPosition = 	new Vector2d(152 - 3*13 - Constants.kCenterToExtendedIntake +  0, 0);	
        Vector2d startPosition = new Vector2d(0,0);;
        Vector2d driveToBallsPosition = new Vector2d(0,0);
        Vector2d intakeBallsPosition = new Vector2d(0,0);
        Vector2d driveToCratePosition = new Vector2d(0,0);
        Vector2d cratePosition = new Vector2d(0,0);
        Vector2d turnAroundPath = new Vector2d(0,0);

        /*
        Path TurnPath = new Path();
		TurnPath.add(new Waypoint(startPosition, tightTurnOptions));	
        TurnPath.add(new Waypoint(driveToBallsPosition, tightTurnOptions));
        TurnPath.setReverseDirection();
        */
        /*
        Path sharpTurnPath = new Path();	
        sharpTurnPath.add(new Waypoint(turnPosition, pathOptions));
        sharpTurnPath.add(new Waypoint(driveToBallsStartPosition, pathOptions));
        sharpTurnPath.setReverseDirection();
        */

        Path driveToBallsPath = new Path();
		driveToBallsPath.add(new Waypoint(driveToBallsPosition, pathOptions));	
        driveToBallsPath.add(new Waypoint(intakeBallsPosition, pathOptions));

        Path ballIntakePath = new Path();
		ballIntakePath.add(new Waypoint(intakeBallsPosition, pathOptions));	
        ballIntakePath.add(new Waypoint(driveToCratePosition, pathOptions));

        /*
        Path turnToCratePath = new Path();
		turnToCratePath.add(new Waypoint(turnToCratePosition, pathOptions));	
        turnToCratePath.add(new Waypoint(driveToCratePosition, pathOptions));
        */

        Path driveToCratePath = new Path();
		driveToCratePath.add(new Waypoint(driveToCratePosition, tightTurnOptions));	
        driveToCratePath.add(new Waypoint(cratePosition, pathOptions));

        Path turnAroundPath = new Path();
		turnAroundPath.add(new Waypoint(cratePosition, tightTurnOptions));	
        turnAroundPath.add(new Waypoint(outtakeStartPosition, tightTurnOptions));
        
        runAction(new goodOuttakeAction()); 
        runAction(new WaitAction(1.0));
        //runAction(new PathFollowerAction(TurnPath));  
        //runAction(new PathFollowerAction(sharpTurnPath));
        runAction(new PathFollowerAction(driveToBallsPath));
        runAction(new ParallelAction (Arrays.asList(new Action[] {
            (PathFollowerAction(ballIntakePath)),
             (IntakeAction())})));
        //runAction(new PathFollowerAction(turnToCratePath));
        runAction(new PathFollowerAction(driveToCratePath)); // don't cross middle line
        runAction(new goodOuttakeStopAction()); 
        runAction(new PathFollowerAction(turnAroundPath));
        runAction(new goodOuttakeAction());      
    }
}
