/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package com.frc2508.cannon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SimpleRobot;

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
       
    /*Initialization*/
    Relay firingSolenoidRelay = new Relay(1, Relay.Direction.kForward);
    Relay tiltRelay = new Relay(2, Relay.Direction.kBoth);
    Relay compressor2Relay = new Relay(3, Relay.Direction.kForward);
    Relay compressor1Relay = new Relay(4, Relay.Direction.kReverse);
    DigitalInput pressureSwitch = new DigitalInput(1);

    Drive robotDrive = new Drive();

    Gamepad gamepad = new Gamepad(1);
    
    /*
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        last = System.currentTimeMillis();
        while (isOperatorControl() && isEnabled()) {
            long millisPerTick = IncrementMillis();
            
            checkPressure(millisPerTick);
            doFire(millisPerTick);
            doTilt(millisPerTick);
            
            robotDrive.DoArcade(millisPerTick, gamepad.getRawAxis(1), gamepad.getRawAxis(2));
        }
        
        robotDrive.ResetRampPair();        
    }
    
    long last;
    private long IncrementMillis(){
        long millisPerTick = System.currentTimeMillis() - last;
        last += millisPerTick;
        if(millisPerTick < 1)millisPerTick=1;
        return millisPerTick;
    }

    private long compressorDelay = 0;
    private long idleDelay = 0;
    private Relay.Value command = Relay.Value.kOff;
    private boolean checkPressure(long millisPerTick) {
        boolean pressureSwitchValue = pressureSwitch.get();
        //System.out.println("Pressure switch state: " + pressureSwitchValue);

        Relay.Value request = !pressureSwitchValue ? Relay.Value.kOn : Relay.Value.kOff;
        
        if(compressorDelay > 0){
            compressorDelay-=millisPerTick;
            if(compressorDelay <= 0){
                compressor1Relay.set(request);
            }
        }
        if(idleDelay > 0) idleDelay -= millisPerTick;
        
        if(command!=request){
            if(request == Relay.Value.kOff || idleDelay <= 0){
                idleDelay = 120000;
                compressorDelay = 1000;
                command = request;
                compressor2Relay.set(request);
            }
        }

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

        Relay.Value relayValue = leftTrigger && rightTrigger && bButton ? Relay.Value.kOn : Relay.Value.kOff;
        firingSolenoidRelay.set(relayValue);
        if(relayValue == Relay.Value.kOn) idleDelay = 0;

        //System.out.println("Firing relay state: " + firingSolenoidRelay.get().value);        
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {

    }
    
    private void printMoveStickAxis() {
        for (int i = 0; i < 6; i++) {
            System.out.println("Axis:" + i + " Value:" + gamepad.getRawAxis(i));
        }
    }
}
