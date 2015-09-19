/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.frc2508.cannon;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author bjohnson
 */
public class Jag extends CANJaguar
{
    public Jag(int deviceNumber) throws CANTimeoutException {
        super(deviceNumber, CANJaguar.ControlMode.kSpeed);
        
            this.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            this.setSafetyEnabled(false);
            //jag.configMaxOutputVoltage(100);
            this.configEncoderCodesPerRev(360);
            this.setPID(.12, .02, .05);
            this.enableControl();
    }
    
    public static Jag CreateJag(int canAddress) {
        Jag jag;
         try{
            jag = new Jag(canAddress);
        }
        catch (CANTimeoutException ex){
            ex.printStackTrace();
            jag = null;
        }   
         
        return jag;
    }    
}
