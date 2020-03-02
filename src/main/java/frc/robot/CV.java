/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @param yez
 */
public class CV {
    private double ball_x;
    private double ball_y;

    private static CV instance;
    public double distance;

    private CV() {
    }

    public static CV getInstance() {
        if (instance == null) {
            instance = new CV();
        }
        return instance;
    }

    public double getBallX() {
        return ball_x;
    }

    public double getBallY() {
        return ball_y;
    }

    public double getDistance() {
        return distance;
        
    }
}
