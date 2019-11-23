package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Test", group = "")
public class LiftTest extends LinearOpMode {

    DcMotor liftLeft;
    DcMotor liftRight;

    @Override
    public void runOpMode() throws InterruptedException {

        liftRight = hardwareMap.get(DcMotor.class, "LLRight");
        liftLeft = hardwareMap.get(DcMotor.class, "LLLeft");

        waitForStart();

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!isStopRequested()){
            double power = 0.0;

            if(gamepad1.dpad_up){
                power = 1.0;
            }else if (gamepad1.dpad_down){
                power = -1.0;
            }else{
                power = 0.0;
            }

            liftRight.setPower(power);
            liftLeft.setPower(-power);

            telemetry.addData("Right", liftRight.getCurrentPosition());
            telemetry.addData("Left", liftLeft.getCurrentPosition());



            telemetry.update();
        }
    }
}
