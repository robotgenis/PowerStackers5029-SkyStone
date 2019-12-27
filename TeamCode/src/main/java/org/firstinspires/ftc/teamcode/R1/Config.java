package org.firstinspires.ftc.teamcode.R1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Config{

    LinearOpMode opMode;
    Drive drive;
    Lift lift;
    Collector collector;

    public Config(LinearOpMode linearOpMode){
        opMode = linearOpMode;
        drive = new Drive();
        lift = new Lift();
        collector = new Collector();

    }

    class Drive{
        DcMotor bl;
        DcMotor br;
        DcMotor fl;
        DcMotor fr;

        BNO055IMU imu;
        double robotAngle =0;

        public Drive(){
            bl = opMode.hardwareMap.get(DcMotor.class, "bl");
            br = opMode.hardwareMap.get(DcMotor.class, "br");
            fl = opMode.hardwareMap.get(DcMotor.class, "fl");
            fr = opMode.hardwareMap.get(DcMotor.class, "fr");

            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);

            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void set(double forward, double right, double rotRight){
            fl.setPower(forward - right - rotRight);
            bl.setPower(forward + right - rotRight);
            fr.setPower(forward + right + rotRight);
            br.setPower(forward - right + rotRight);
        }

        public void setAuto(double forward, double right, double rotRight, boolean correction){
            double rot = imu.getAngularOrientation().firstAngle - robotAngle;
            rot += (rot > 180) ? -360 : (rot < -180) ? 360 : 0;

            rot = (rot > 1) ? .1 : (rot < -1) ? -.1 : 0;

            set(forward, right, rotRight + rot);
        }

        public void stop(){
            fl.setPower(0.0);
            fr.setPower(0.0);
            bl.setPower(0.0);
            br.setPower(0.0);
        }

        public void initGyro(){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            drive.imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
            drive.imu.initialize(parameters);
        }

        double trackerWheelCalibration = 0.0;
        double wheelDiameter = 3;
        double ticksPerRotation = 4096;

        public void initTrackerWheel(){
            //drive.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetTrackerWheelPosition();
        }

        public void resetTrackerWheelPosition(){
            trackerWheelCalibration = drive.br.getCurrentPosition();
        }

        public double getTrackerWheelPosition(){
            return drive.br.getCurrentPosition() - trackerWheelCalibration;
        }

        public double getDistanceTraveled(){
            return (getTrackerWheelPosition() / ticksPerRotation) * Math.PI * wheelDiameter;
        }

        public boolean travelDistance(double speed, double distance){
            double currentPosition = getDistanceTraveled();
            double disToTarget = distance - currentPosition;
            double maxSpeed = Math.abs(disToTarget / 10);
            double maxSpeed2 = Math.abs(currentPosition / 10) + .2;
            if(speed > maxSpeed){
                speed = maxSpeed;
            }
            if(speed > maxSpeed2){
                speed = maxSpeed2;
            }

            speed *= (disToTarget < 0) ? -1 : 1;

            opMode.telemetry.addData("Distance TO Target", disToTarget);

            if(Math.abs(speed) < .1){
                stop();
                return true;
            }
            setAuto(-speed, 0, 0, true);
            return false;
        }

        public boolean turnToAngle(double speed, double angle){
            robotAngle = angle;

            double currentAngle = imu.getAngularOrientation().firstAngle;
            double change = angle - currentAngle;
            change += (change > 180) ? -360 : (change < -180) ? 360 : 0;
            double maxSpeed = Math.abs(change / 90) + 0.05;
            if(speed > maxSpeed){
                speed = maxSpeed;
            }
            speed *= (change > 0) ? -1 : 1;

            if(Math.abs(speed) < .1){
                stop();
                return true;
            }
            setAuto(0, 0, speed, false);
            return false;
        }
    }

    class Lift{
        DcMotor r;
        DcMotor l;

        Servo g;
        Servo s;

        DistanceSensor sensorDistance;

        public Lift(){
            r = opMode.hardwareMap.get(DcMotor.class, "lr");
            l = opMode.hardwareMap.get(DcMotor.class, "ll");
            r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            g = opMode.hardwareMap.get(Servo.class, "g");

            s = opMode.hardwareMap.get(Servo.class, "s");

            sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "CD");
        }
    }

    class Collector{
        DcMotor r;
        DcMotor l;

        public Collector(){
            r = opMode.hardwareMap.get(DcMotor.class, "cr");
            l = opMode.hardwareMap.get(DcMotor.class, "cl");
        }
    }
}
