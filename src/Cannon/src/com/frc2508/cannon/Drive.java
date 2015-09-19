/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.frc2508.cannon;

import edu.wpi.first.wpilibj.RobotDrive;

/**
 *
 * @author bjohnson
 */
public class Drive extends RobotDrive{
      
    /*Tunables*/
    final double deadBand = .05;    
 
    public Drive() {
        //Front Left, Rear Left, Front Right, Rear Right
        super(Jag.CreateJag(2), Jag.CreateJag(8), Jag.CreateJag(3), Jag.CreateJag(4));
        
        this.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);

        double gearRatio = 12.0;
        double maxRPM = 5310.0;
        this.setMaxOutput((maxRPM / gearRatio) / 1.5);
    }
    
    public void DoArcade(long millisPerTick, double inRotate, double inMove) {
        inRotate = -1 * inRotate;
        inMove = -1 * inMove;
        
        Pair pair = squareTheCircle(doTimeRamp(millisPerTick, new Pair(inRotate, inMove)));
        double outX = deadband(pair.x/2);
        double outY = deadband(pair.y);
        
        this.arcadeDrive(outX, outY, true);
    }
    
    private void DoMecanum(long millisPerTick, double inX, double inY, double inTwist) {

        double outX = deadband(inX);
        double outY = deadband(inY);

        double invertedOutY = -1 * outY;
        double invertedTwist = -1 * inTwist;

        outX = ramp(outX);
        // outY = ramp(outY);
        invertedOutY = ramp(invertedOutY);
        invertedTwist = ramp(invertedTwist);

        this.mecanumDrive_Cartesian(outX, invertedOutY, invertedTwist, 0);
    }

    public void DoTank(long millisPerTick, double inLeft, double inRight) {        
        inRight = -1 * inRight;

        double outLeft = ramp(deadband(inLeft));
        double outRight = ramp(deadband(inRight));

        this.tankDrive(outLeft, outRight);
    }
    
    public void ResetRampPair()
    {
        rampPairThing = new Pair(0,0);
    }
    
    Pair rampPairThing = new Pair(0,0);
    double slewRate = .10;
    
    private Pair doTimeRamp(long millisPerTick, Pair pair)
    {
        double posSlewLimit = slewRate*1000/millisPerTick;
        double negSlewLimit = -posSlewLimit;
        double xDiff = pair.x - rampPairThing.x;
        double yDiff = pair.y - rampPairThing.y;
       
        xDiff = Math.max(negSlewLimit, Math.min(posSlewLimit, xDiff));
        yDiff = Math.max(negSlewLimit, Math.min(posSlewLimit, yDiff));
        
        rampPairThing = new Pair(rampPairThing.x + xDiff, rampPairThing.y + yDiff);
        
        return rampPairThing;
    }
    
    public static double ramp(double input) {
        if (input == 0) {
            return 0;
        }

        return input * Math.abs(input);
    }

    private double deadband(double input) {        
        return input > deadBand || input < -1 * deadBand ? input : 0;
    }    
    	
    private Pair squareTheCircle(Pair input)
    {
        double shortSide = Math.min(Math.abs(input.x),Math.abs(input.y));
        double longSide = Math.max(Math.abs(input.x),Math.abs(input.y));

        double r = Math.sqrt((shortSide*shortSide)+(longSide*longSide));

        double scaleFactor = (longSide/r);

        return new Pair(input.x/scaleFactor,input.y/scaleFactor);
    }
}