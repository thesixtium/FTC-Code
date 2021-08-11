package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="DEREK DRIVE - HAS FOUNDATION v11", group="THIS COMP")
@Disabled

public class TeleOpDrivingWithPlatform extends LinearOpMode {

    //Hardware Initializing
      private  Servo                    platform;

    private  DcMotor                  frontLeftMotor;
    private  DcMotor                  frontRightMotor;
    private  DcMotor                  backLeftMotor;
    private  DcMotor                  backRightMotor;

    private  DcMotor                  leftIntakeWheel;
    private  DcMotor                  rightIntakeWheel;


    //Variable Initializing
    double speedMultiplier;


    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT

        //Variable Initialization

        //Hardware Mapping
        platform = hardwareMap.servo.get("platform");

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        leftIntakeWheel = hardwareMap.dcMotor.get("LeftIntakeWheel");
        rightIntakeWheel = hardwareMap.dcMotor.get("RightIntakeWheel");


        //Hardware Initialization
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set variables
        speedMultiplier = 1;


        //Wait for start
        telemetry.addData("Status: ", "Done");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP


                foundationControls();
                slowModeDriving();
                speedMultiplier = 1;
                intakeWheels();


                //TELEMTRY
                int frontLeftMotorDisplayPower   =  (int) Math.round(frontLeftMotor.getPower()  );
                int frontRightMotorDisplayPower  =  (int) Math.round(frontRightMotor.getPower() );
                int backLeftMotorDisplayPower    =  (int) Math.round(backLeftMotor.getPower()   );
                int backRightMotorDisplayPower   =  (int) Math.round(backRightMotor.getPower()  );

                telemetry.addLine( ( "FL: "  + frontLeftMotorDisplayPower   +  "  FR: " + frontRightMotorDisplayPower    ) );
                telemetry.addLine( ( "BL: "  + backLeftMotorDisplayPower    +  "  BR: " + backRightMotorDisplayPower     ) );
    //            telemetry.addLine( ( "LFS :" + leftFrontClamp.getPosition() + "  RFS :" + rightFrontClamp.getPosition()  ) );
                telemetry.addLine( ("Carson's Gay Magnet: "+"True" + "    Divya + Carson: " + "Viable"                  ) );
      //          telemetry.addLine( ("CS: " + capstoneServo.getPosition() + "ZAS: " + zAxisClawSpinSevo.getPosition()     ) );
                telemetry.update();

            }
        }
    }


    //Functions
    public void slowModeDriving() {
        if (this.gamepad1.dpad_up) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
        } else if (this.gamepad1.dpad_down) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(1);
        } else if (this.gamepad1.dpad_left) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
        } else if (this.gamepad1.dpad_right) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(1);
        } else if (this.gamepad1.left_bumper) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
        } else if (this.gamepad1.right_bumper) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(1);
        } else {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }


    public void intakeWheels() {
        float wheelPower = (this.gamepad1.left_trigger - this.gamepad1.right_trigger);
        leftIntakeWheel.setPower(wheelPower*0.5);
        rightIntakeWheel.setPower(wheelPower*0.5);
    }

   public void foundationControls(){
        if (this.gamepad1.x){
            platform.setPosition(0);
        } else if (this.gamepad1.y) {
            platform.setPosition(0.55);
        }
    }
}