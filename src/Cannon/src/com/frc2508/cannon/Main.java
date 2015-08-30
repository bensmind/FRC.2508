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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.command.Command;

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
    
    Relay firingSolenoidRelay;
        
    CANJaguar[] motors = new CANJaguar[4];

    RobotDrive robotDrive;

    Gamepad gamepad;
    /*
     * This function is called once each time the robot enters operator control.
     */

    private void configJaguar(int index, int id) {
        try {
            CANJaguar jag = new CANJaguar(id, CANJaguar.ControlMode.kSpeed);
            jag.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            jag.setSafetyEnabled(false);
            //jag.configMaxOutputVoltage(100);
            jag.configEncoderCodesPerRev(360);
            jag.setPID(.12, .02, 0);
            jag.enableControl();

            motors[index] = jag;
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    public void configureJaguars() {
        configJaguar(0, 8);
        configJaguar(1, 2);
        configJaguar(2, 3);
        configJaguar(3, 4);
    }
    
    public void configureSpikes(){
        firingSolenoidRelay = new Relay(1, Relay.Direction.kForward);
    }

    public void operatorControl() {
        configureJaguars();
        configureSpikes();
        
        gamepad = new Gamepad(1);

        robotDrive = new RobotDrive(motors[1], motors[0], motors[2], motors[3]);

        robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);

        double gearRatio = 12.0;
        double maxRPM = 5310.0;
        robotDrive.setMaxOutput((maxRPM / gearRatio) / 4.5);

        while (isOperatorControl() && isEnabled()) {
            //doMecanum();
            //doTank();
            boolean aButton = gamepad.getButtonA().get();
            System.out.println("A Button state: " + aButton);
            
            if(aButton)
            {
                firingSolenoidRelay.set(Relay.Value.kOn);
            }
            else
            {
                firingSolenoidRelay.set(Relay.Value.kOff);
            }
            System.out.println("Relay state: " + firingSolenoidRelay.get().value);           
     
            doArcade();
            printMoveStickAxis();

            Timer.delay(.01);
        }
    }

    public double ramp(double input) {
        if (input == 0) {
            return 0;
        }

        return input * Math.abs(input);
    }

    public double deadband(double input) {
        double deadBand = .05;
        return input > deadBand || input < -1 * deadBand ? input : 0;
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {

    }

    private void doMecanum() {
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

    private void doTank() {
        double inLeft = gamepad.getRawAxis(2);
        double inRight = -gamepad.getRawAxis(4);

        double outLeft = ramp(deadband(inLeft));
        double outRight = ramp(deadband(inRight));

        robotDrive.tankDrive(outLeft, outRight);
    }
    private void doArcade() {
        double inMove = gamepad.getRawAxis(1);
        double inRotate = gamepad.getRawAxis(2);
        inRotate = -1 * inRotate;
        
        robotDrive.arcadeDrive(inMove,inRotate,true);
    }

    private void printMoveStickAxis() {
        for (int i = 0; i < 6; i++) {
            System.out.println("Axis:" + i + " Value:" + gamepad.getRawAxis(i));
        }
    }
}
