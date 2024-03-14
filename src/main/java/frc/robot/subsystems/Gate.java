/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gate extends SubsystemBase {
    private Spark gateMotor;
    private float speed = 0;

    /**
     * Creates a new Gate.
     */

    public Gate() {
        gateMotor = new Spark(Constants.gateSparkPort);
        closeGate();
    }

    public void openGate() {
        speed = -1;
    }

    public void closeGate() {
        speed = 1;
    }

    public void stop() {speed = 0; }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        gateMotor.set(speed);
    }
}
