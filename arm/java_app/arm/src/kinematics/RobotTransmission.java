package kinematics;

public class RobotTransmission {
    // Config constants in Actuator Space (q)
    public static final double Q3_RIGHT_MIN = 0.0;
    public static final double Q3_RIGHT_MAX = 145.0;
    public static final double Q4_RIGHT_MIN_OFFSET = -60.0;
    public static final double Q4_RIGHT_MAX_OFFSET = 20.0;

    public static final double Q3_LEFT_MIN = -145.0;
    public static final double Q3_LEFT_MAX = 0.0;
    public static final double Q4_LEFT_MIN_OFFSET = -20.0;
    public static final double Q4_LEFT_MAX_OFFSET = 60.0;

    /**
     * Converts Joint Space (theta3, theta4) to Actuator Space (q3, q4).
     * Home positions (q = 0) correspond to:
     *   Right: theta3 = 20.0, theta4 = -35.0 (which is q4 = -15.0 relative to theta3)
     *   Left:  theta3 = -20.0, theta4 = 35.0 (which is q4 = 15.0 relative to theta3)
     */
    public static double[] jointToActuator(double theta3, double theta4, boolean isRight) {
        double q3, q4;
        if (isRight) {
            q3 = theta3 - 20.0;
            q4 = theta3 + theta4 + 15.0;
        } else {
            q3 = theta3 + 20.0;
            q4 = theta3 + theta4 - 15.0;
        }
        return new double[]{ q3, q4 };
    }

    /**
     * Converts Actuator Space (q3, q4) to Joint Space (theta3, theta4).
     */
    public static double[] actuatorToJoint(double q3, double q4, boolean isRight) {
        double theta3, theta4;
        if (isRight) {
            theta3 = q3 + 20.0;
            theta4 = q4 - q3 - 35.0;
        } else {
            theta3 = q3 - 20.0;
            theta4 = q4 - q3 + 35.0;
        }
        return new double[]{ theta3, theta4 };
    }
}
