// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public class RobotConstants {

        /**
         * The maximum battery voltage.
         */
        public static final double MAX_BATTERY_VOLTAGE = 12.0;

        // public static final double ROBOT_LENGTH = Units.inchesToMeters(39);

        /**
         * Wheel diameter.
         */
        public static final double WHEEL_DIAMETER_INCHES = 4.0;

        public static class OperatorConstants {

            public static class XboxControllerPort {
                public static final int DRIVER = 0;
                public static final int MANIPULATOR = 1;
            }
        }

        public static class CAN {
            public static class SparkMax {
                public static final int INDEXER_PORT = 58; // TODO assign actual port
                public static final int SHOOTER_PORT = 59; // TODO assign actual port
                public static final int CLIMBER_PORT = 60; // TODO assign actual port
                public static final int INTAKE_PORT = 61; // TODO assign actual port
            }
        }
    }
}
