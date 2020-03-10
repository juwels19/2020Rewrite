/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Constants {

    // Controllers
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.2;
    public static final double OPERATOR_CONTROLLER_DEADBAND = 0.2;

    // Drive
    public static final int DRIVE_LEFT_MASTER_CAN_ID = 0;
    public static final int DRIVE_LEFT_SLAVE_CAN_ID = 1;
    public static final int DRIVE_RIGHT_MASTER_CAN_ID = 2;
    public static final int DRIVE_RIGHT_SLAVE_CAN_ID = 3;

    // Intake/Elevator
    public static final int SIDE_ROLLER_INTAKE_CAN_ID = 5;
    public static final int ELEVATOR_MOTOR_CAN_ID = 6;
    public static final int DINGUS_MOTOR_CAN_ID = 10;

    // Shooter
    public static final int LEFT_SHOOTER_CAN_ID = 7;
    public static final int RIGHT_SHOOTER_CAN_ID = 8;

    // Pneumatics
    public static final int INTAKE_CYLINDERS_FORWARD_PORT = 0;
    public static final int INTAKE_CYLINDERS_REVERSE_PORT = 1;
    public static final int TENSIONER_CYLINDERS_FORWARD_PORT = 2;
    public static final int TENSIONER_CYLINDERS_REVERSE_PORT = 3;

    // LEDs
    public static final int LED_PWM_PORT = 0;
    public static final int LED_COUNT = 40;

    // Limelight
    public static final double LIMELIGHT_YAW_THRESHOLD = 1.0;
    public static final double LIMELIGHT_THROTTLE_THRESHOLD = 1.0;

}
