package org.firstinspires.ftc.teamcode.system;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DifferentialSwerveModule {
    private final DcMotorEx topMotor;
    private final DcMotorEx bottomMotor;

    private final double x;
    private final double y;

    private final double rotationCenterDist;
    private final double turnDirX;
    private final double turnDirY;

    // All passed in distance variables should be in this unit
    private final DistanceUnit distUnit = DistanceUnit.INCH;
    private final double WHEEL_RADIUS = 1.8898;

    private final double COUNTS_PER_REVOLUTION = 383.6;

    // How many times the differential rotates for every spin of the wheel
    private final double DIFFERENTIAL_TO_WHEEL_GEAR_RATIO = 1.0 / 4.0;

    // How many times the motor rotates for every spin of the differential;
    private final double MOTOR_TO_DIFFERENTIAL_GEAR_RATIO = 4;
    private final double MAX_MOTOR_VELOCITY = 2800;

    private final PIDFController angleController;

    private double wantedAngle;
    private double topMotorVelocity;
    private double bottomMotorVelocity;

    public DifferentialSwerveModule(
            DcMotorEx topMotor,
            DcMotorEx bottomMotor,
            double x,
            double y
    ) {
        topMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Have to be reversed if both motors are coming from the top
        topMotor.setDirection(DcMotorEx.Direction.REVERSE);
        bottomMotor.setDirection(DcMotorEx.Direction.REVERSE);

        topMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;

        angleController = new PIDFController(1, 0, 0, 0);
        angleController.reset();

        this.x = x;
        this.y = y;

        rotationCenterDist = Math.hypot(x, y);

        turnDirX = -y / rotationCenterDist;
        turnDirY = x / rotationCenterDist;
    }

    public void setAnglePIDF(
            double p,
            double i,
            double d,
            double f
    ) {
        angleController.setPIDF(p, i, d, f);
        angleController.reset();
    }

    public void resetPIDF() {
        angleController.reset();
    }

    public double getAngle() {
        return normalize((180 * (topMotor.getCurrentPosition() + bottomMotor.getCurrentPosition())) / (COUNTS_PER_REVOLUTION * MOTOR_TO_DIFFERENTIAL_GEAR_RATIO));
    }

    public void setAngle(
            double angle
    ) {
        wantedAngle = normalize(angle);
    }

    // Sets speed of the swerve module, should be same unit as distUnit / s
    public void setSpeed(
            double speed
    ) {
        double wheelRPS = speed / (2 * Math.PI * WHEEL_RADIUS);
        double differentialGearRPS = wheelRPS * DIFFERENTIAL_TO_WHEEL_GEAR_RATIO;
        double motorRPS = differentialGearRPS * MOTOR_TO_DIFFERENTIAL_GEAR_RATIO;
        double motorVel = motorRPS * COUNTS_PER_REVOLUTION;

        topMotorVelocity = motorVel;
        bottomMotorVelocity = -motorVel;
    }

    // Accepts a Pose2D velocity object, with the x and y being the drive velocity and the heading
    // being the turn velocity. Positive turn velocity = turn left
    public void setRobotVelocity(
            Pose2D velocity
    ) {
        double wheelTurnVelocityMagnitude = velocity.getHeading(AngleUnit.RADIANS) * rotationCenterDist;
        double wheelVelocityX = turnDirX * wheelTurnVelocityMagnitude + velocity.getX(distUnit);
        double wheelVelocityY = turnDirY * wheelTurnVelocityMagnitude + velocity.getY(distUnit);

        double speed = Math.hypot(wheelVelocityX, wheelVelocityY);
        double angle = Math.toDegrees(Math.atan2(wheelVelocityY, wheelVelocityX));
        double currentAngle = getAngle();

        if (Math.abs(currentAngle - angle) > 90) {
            angle = normalize(angle + 180);
            speed *= -1;
        }

        setSpeed(speed);
        setAngle(angle);
    }

    public void drive() {
        double correction = angleController.calculate(0, normalize(wantedAngle - getAngle()));
        double correctedTopVelocity = topMotorVelocity + correction;
        double correctedBottomVelocity = bottomMotorVelocity + correction;

        double maxVelocity = Math.max(Math.abs(correctedTopVelocity), Math.abs(correctedBottomVelocity));

        if (maxVelocity > MAX_MOTOR_VELOCITY) {
            double velMult = MAX_MOTOR_VELOCITY / maxVelocity;
            correctedTopVelocity *= velMult;
            correctedBottomVelocity *= velMult;
        }

        topMotor.setVelocity(correctedTopVelocity);
        bottomMotor.setVelocity(correctedBottomVelocity);
    }

    private static double normalize(
            double degrees
    ) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }
}
