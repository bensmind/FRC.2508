/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.first.team2508.armada.shirt.cannon;


import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.SimpleRobot;
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

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        CANJaguar jag1;
      try {
        jag1 = new CANJaguar(1, CANJaguar.ControlMode.kSpeed);
        jag1.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
        jag1.setSafetyEnabled(false);
        //jag1.configMaxOutputVoltage(100);
        jag1.configEncoderCodesPerRev(250);
        jag1.setPID(.1, .05, 0);
        jag1.enableControl();
        double speed = 50; 
        while (isOperatorControl() && isEnabled()) {
            if(speed > 1000){
              speed = -1000;
            }
          
            jag1.setX(speed);
            speed++;
            Thread.sleep(10);
          }
      } catch (CANTimeoutException ex) {
        ex.printStackTrace();
      } catch (InterruptedException ex) {
        ex.printStackTrace();
      }

    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
