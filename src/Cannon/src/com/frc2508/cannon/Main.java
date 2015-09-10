/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package com.frc2508.cannon;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Main extends SimpleRobot {

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {

    }
    
    /*Tunables*/
    double deadBand = .05;
    
    /*Initialization*/
    Relay firingSolenoidRelay = new Relay(1, Relay.Direction.kForward);
    Relay tiltRelay = new Relay(2, Relay.Direction.kBoth);
    Relay compressor2Relay = new Relay(3, Relay.Direction.kForward);
    Relay compressor1Relay = new Relay(4, Relay.Direction.kReverse);
    DigitalInput pressureSwitch = new DigitalInput(1);

    CANJaguar[] driveMotors = new CANJaguar[4];

    RobotDrive robotDrive;

    Gamepad gamepad = new Gamepad(1);

    private void configJaguar(int index, int id) {
        try {
            CANJaguar jag = new CANJaguar(id, CANJaguar.ControlMode.kSpeed);
            jag.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            jag.setSafetyEnabled(false);
            //jag.configMaxOutputVoltage(100);
            jag.configEncoderCodesPerRev(360);
            jag.setPID(.12, .02, 0);
            jag.enableControl();

            driveMotors[index] = jag;
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    public void configureDrive() {
        configJaguar(0, 8);
        configJaguar(1, 2);
        configJaguar(2, 3);
        configJaguar(3, 4);
        robotDrive = new RobotDrive(driveMotors[1], driveMotors[0], driveMotors[2], driveMotors[3]);
        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);

        double gearRatio = 12.0;
        double maxRPM = 5310.0;
        robotDrive.setMaxOutput((maxRPM / gearRatio) / 50);
    }

    /*
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        configureDrive();
        long last = System.currentTimeMillis();
        while (isOperatorControl() && isEnabled()) {
            long millisPerTick = System.currentTimeMillis() - last;
            last += millisPerTick;
            //doMecanum(millisPerTick);
            //doTank(millisPerTick);   
            checkPressure(millisPerTick);
            doFire(millisPerTick);
            doTilt(millisPerTick);
            doArcade(millisPerTick);
            //printMoveStickAxis();

            //Timer.delay(.01);
        }
    }

    private long compressorDelay = 1000;
    private boolean checkPressure(long millisPerTick) {
        boolean pressureSwitchValue = pressureSwitch.get();
        //System.out.println("Pressure switch state: " + pressureSwitchValue);

        Relay.Value relayValue = !pressureSwitchValue ? Relay.Value.kOn : Relay.Value.kOff;

        if(compressorDelay <= (1000 * millisPerTick) & compressorDelay > 0)
        {
            compressorDelay--;
        }
        else
        {
            compressorDelay = 1000;
        }
        
        Relay.Value compressor2Value;
        if(relayValue == Relay.Value.kOn && compressorDelay == 0)
        {
            compressor2Value =  Relay.Value.kOn;
        }
        else
        {
            compressor2Value = Relay.Value.kOff;
        }
        
        compressor1Relay.set(relayValue);
        compressor2Relay.set(compressor2Value);


        //System.out.println("Compressor 1 relay state: " + compressor1Relay.get().value);    
        //System.out.println("Compressor 2 relay state: " + compressor2Relay.get().value);    
        return pressureSwitchValue;
    }
    
    private void doTilt(long millisPerTick){
        boolean upButton = gamepad.getButtonStateX();
        boolean downButton = gamepad.getButtonStateA();
        if(upButton){
            tiltRelay.set(Relay.Value.kForward);
        }
        else if(downButton)
        {
            tiltRelay.set(Relay.Value.kReverse);
        }
        else
        {
            tiltRelay.set(Relay.Value.kOff);
        }
    }

    private void doFire(long millisPerTick) {
        boolean bButton = gamepad.getButtonStateB();
        boolean leftTrigger = gamepad.getLeftTriggerClick().get();
        boolean rightTrigger = gamepad.getRightTriggerClick().get();

//        System.out.println("Left trigger state: " + leftTrigger);
//        System.out.println("Right trigger state: " + rightTrigger);

        Relay.Value relayValue = leftTrigger && rightTrigger && bButton ? Relay.Value.kOn : Relay.Value.kOff;
        firingSolenoidRelay.set(relayValue);

        //System.out.println("Firing relay state: " + firingSolenoidRelay.get().value);        
    }

    public static double ramp(double input) {
        if (input == 0) {
            return 0;
        }

        return input * Math.abs(input);
    }

    public double deadband(double input) {        
        return input > deadBand || input < -1 * deadBand ? input : 0;
    }    
    	
    public Pair squareTheCircle(Pair input)
    {
        double shortSide = Math.min(Math.abs(input.x),Math.abs(input.y));
        double longSide = Math.max(Math.abs(input.x),Math.abs(input.y));

        double r = Math.sqrt((shortSide*shortSide)+(longSide*longSide));

        double scaleFactor = (longSide/r);

        return new Pair(input.x/scaleFactor,input.y/scaleFactor);
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {

    }    	

    private void doMecanum(long millisPerTick) {
        double inX = gamepad.getX();
        double inY = gamepad.getY();
        double inTwist = gamepad.getTwist();

        double outX = deadband(inX);
        double outY = deadband(inY);

        double invertedOutY = -1 * outY;
        double invertedTwist = -1 * inTwist;

        outX = ramp(outX);
        // outY = ramp(outY);
        invertedOutY = ramp(invertedOutY);
        invertedTwist = ramp(invertedTwist);

        robotDrive.mecanumDrive_Cartesian(outX, invertedOutY, invertedTwist, 0);
    }

    private void doTank(long millisPerTick) {
        double inLeft = gamepad.getRawAxis(2);
        double inRight = -gamepad.getRawAxis(4);

        double outLeft = ramp(deadband(inLeft));
        double outRight = ramp(deadband(inRight));

        robotDrive.tankDrive(outLeft, outRight);
    }

    private void doArcade(long millisPerTick) {
        double inRotate = gamepad.getRawAxis(1);
        inRotate = -1 * inRotate;
        
        double inMove = gamepad.getRawAxis(2);
        inMove = -1 * inMove;
        
        Pair pair = squareTheCircle(doTimeRamp(millisPerTick,new Pair(inRotate, inMove)));
        
        robotDrive.arcadeDrive(deadband(pair.x), deadband(pair.y), true);
    }
    
    Pair rampPairThing = new Pair(0,0);
    double slewRate = 1.0;
    
    private Pair doTimeRamp(long millisPerTick, Pair pair)
    {
        double posSlewLimit = slewRate*1000/millisPerTick;
        double negSlewLimit = -posSlewLimit;
        double xDiff = rampPairThing.x - pair.x;
        double yDiff = rampPairThing.y - pair.y;
       
        xDiff = Math.max(negSlewLimit, Math.min(posSlewLimit, xDiff));
        yDiff = Math.max(negSlewLimit, Math.min(posSlewLimit, yDiff));
        
        rampPairThing = new Pair(rampPairThing.x + xDiff, rampPairThing.y + yDiff);
        
        return rampPairThing;
    }

    private void printMoveStickAxis() {
        for (int i = 0; i < 6; i++) {
            System.out.println("Axis:" + i + " Value:" + gamepad.getRawAxis(i));
        }
    }
}
