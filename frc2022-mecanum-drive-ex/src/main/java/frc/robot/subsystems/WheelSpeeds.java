package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class WheelSpeeds {
    public double frontLeft;
    public double frontRight;
    public double rearLeft;
    public double rearRight;

    /** Constructs a WheelSpeeds with zeroes for all four speeds. */
    public WheelSpeeds() {
    }

    /**
     * Constructs a WheelSpeeds.
     *
     * @param frontLeft  The front left speed [-1.0..1.0].
     * @param frontRight The front right speed [-1.0..1.0].
     * @param rearLeft   The rear left speed [-1.0..1.0].
     * @param rearRight  The rear right speed [-1.0..1.0].
     * 
     */

    public WheelSpeeds(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
    }

    public static WheelSpeeds driveCartesianIK(
            double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

        // Compensate for gyro angle.
        Vector2d input = new Vector2d(ySpeed, xSpeed);
        input.rotate(-gyroAngle);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = input.x + input.y + zRotation;
        wheelSpeeds[MotorType.kFrontRight.value] = input.x - input.y - zRotation;
        wheelSpeeds[MotorType.kRearLeft.value] = input.x - input.y + zRotation;
        wheelSpeeds[MotorType.kRearRight.value] = input.x + input.y - zRotation;

        normalize(wheelSpeeds);

        return new WheelSpeeds(
                wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value],
                wheelSpeeds[MotorType.kRearLeft.value],
                wheelSpeeds[MotorType.kRearRight.value]);
    }

    protected static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

}
