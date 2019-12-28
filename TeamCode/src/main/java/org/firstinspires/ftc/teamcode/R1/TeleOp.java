package org.firstinspires.ftc.teamcode.R1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "R1")
public class TeleOp extends LinearOpMode {

    Config robot;

    double slidePosition = 0.95;
//    double rightSlidePosition = 0.5;
    double grabberPosition = 0.1;

    double collectorPower = 0;
    boolean collectorOn = false;

    double driveScale = 1;

    boolean lowerLift = false;

    ElapsedTime timer = new ElapsedTime();

    boolean up = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Config(this);

        timer.reset();

        waitForStart();

        updateSlideGrabber();

        while(!isStopRequested()){
            //set drive
            robot.drive.set(gamepad1.left_stick_y * driveScale , gamepad1.left_stick_x * driveScale, gamepad1.right_stick_x * driveScale);

            //collector
            if(gamepad1.right_trigger > collectorPower){
                collectorPower = (gamepad1.right_trigger > 0.85) ? 0.85 : gamepad1.right_trigger;
                collectorOn = true;
                grabberPosition = 0.3;
                updateSlideGrabber();
            }
            if(gamepad2.right_trigger > collectorPower) {
                collectorPower = (gamepad2.right_trigger > 0.85) ? 0.85 : gamepad2.right_trigger;
                collectorOn = true;
                grabberPosition = 0.3;
                updateSlideGrabber();
            }
            if(gamepad1.left_trigger > .1 || gamepad1.left_trigger > .1){
                collectorOn = false;
            }
            if(!collectorOn) {
                double value = gamepad1.left_trigger + gamepad1.left_trigger;
                collectorPower = -(value > .2 ? .2 : value);
            }
            robot.collector.l.setPower(collectorPower);
            robot.collector.r.setPower(-collectorPower);
            telemetry.addData("collector", collectorPower);

            if(gamepad1.a || gamepad2.a){
                grabberPosition = 0.9;
                collectorOn = false;
                updateSlideGrabber();
            }
            if(gamepad1.b || gamepad2.b){
                slidePosition = 0.18;
                driveScale = .5;
                updateSlideGrabber();
            }
            if(gamepad1.y || gamepad2.y){
                grabberPosition = 0.3;
                updateSlideGrabber();
            }

            if(gamepad1.left_bumper){
                grabberPosition = 0.3;
                updateSlideGrabber();
                timer.reset();
            }
            if(gamepad1.x || gamepad2.x){
                slidePosition = 0.95;
                driveScale = 1;
                if(grabberPosition == 0.3){
                    grabberPosition = 0.4;
                }
                updateSlideGrabber();
            }
            if(timer.milliseconds() > 250 && timer.milliseconds() < 500){
                slidePosition = 0.95;
                slidePosition = 0.95;
                driveScale = 1;
                grabberPosition = 0.4;
                updateSlideGrabber();
            }

            double sensorDistance = robot.lift.sensorDistance.getDistance(DistanceUnit.MM);

            if(gamepad1.dpad_up || gamepad2.dpad_up){
                lowerLift = false;
            }

            if(sensorDistance > 100 && (gamepad1.right_bumper || gamepad2.right_bumper) && slidePosition > .6){
                lowerLift = true;
                grabberPosition = .9;
                updateSlideGrabber();
            }
            double liftPower = 0;
            if(lowerLift){
                if(sensorDistance < 100){
                    liftPower = 0;
                    grabberPosition = .3;
                    updateSlideGrabber();
                    lowerLift = false;
                }else {
                    liftPower = -.4;
                }
            } else{
                if(gamepad1.dpad_up || gamepad2.dpad_up){
                    up = true;
                    liftPower = 1.0;
                }else if(gamepad1.dpad_down || gamepad2.dpad_down){
                    up = false;
                    liftPower = -0.8;
                }else{
                    if(up){
                        liftPower = .1;
                    }else{
                        liftPower = 0;
                    }
                }
                robot.lift.l.setPower(liftPower);
                robot.lift.r.setPower(-liftPower);
                telemetry.addData("lift" , liftPower);
            }

            robot.lift.l.setPower(liftPower);
            robot.lift.r.setPower(-liftPower);

            telemetry.addData("slide", slidePosition);
            telemetry.addData("grabber", grabberPosition);

            telemetry.addData("Distance Sensor", robot.lift.sensorDistance.getDistance(DistanceUnit.MM));

            telemetry.update();

            //Motor Identification helper
//            if(gamepad1.dpad_up){
//                if(gamepad1.dpad_left) {
//                    robot.drive.fl.setPower(.5);
//                }else if (gamepad1.dpad_right){
//                    robot.drive.fr.setPower(.5);
//                }
//            }
//            if(gamepad1.dpad_down){
//                if(gamepad1.dpad_left) {
//                    robot.drive.bl.setPower(.5);
//                }else if (gamepad1.dpad_right){
//                    robot.drive.br.setPower(.5);
//                }
//            }
        }
    }

    public void updateSlideGrabber(){
        robot.lift.g.setPosition(grabberPosition);
        robot.lift.s.setPosition(slidePosition);
    }
}
