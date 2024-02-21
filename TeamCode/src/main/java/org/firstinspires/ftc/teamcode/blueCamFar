//R
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
public class blueCamFar extends LinearOpMode {

    /* Declare OpMode members. */
    // private DcMotor fL = null;
    // private DcMotor fR = null;
    // private DcMotor bL = null;
    // private DcMotor bR = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private Servo claw = null;
    private Servo clawTwo = null;
    private Servo flippyL = null;
    private Servo flippyR = null;
    //private Servo turnyL = null;
    private Servo turnyR = null;
    private DcMotor armL = null;
    private DcMotor armR = null;
    private Servo woosh = null;
    OpenCvWebcam webcam;
    pictureTimeBlue pipeline;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        claw = hardwareMap.get(Servo.class, "claw");
        clawTwo = hardwareMap.get(Servo.class, "clawTwo");
        flippyL = hardwareMap.get(Servo.class, "flippyL");
        flippyR = hardwareMap.get(Servo.class, "flippyR");
        //turnyL = hardwareMap.get(Servo.class, "turnyL");
        turnyR = hardwareMap.get(Servo.class, "turnyR");
        armL = hardwareMap.get(DcMotor.class, "armL");
        armR = hardwareMap.get(DcMotor.class, "armR");
        woosh = hardwareMap.get(Servo.class, "woosh");
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        armL.setDirection(DcMotor.Direction.REVERSE);
        armR.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new pictureTimeBlue();
        webcam.setPipeline(pipeline);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("erroCode", errorCode);
            }
        });
        waitForStart();
        // telemetry.addData("Analysis", pipeline.getAnalysis());
        // telemetry.update();
        int barkAndBark = pipeline.getAnalysis();
        if(barkAndBark == 1) {
            telemetry.addData("works", 1);
        }
        if(barkAndBark == 2) {
            telemetry.addData("works", 2);
        }
        if(barkAndBark == 3) {
            telemetry.addData("works", 3);
        }
        if(barkAndBark == 0) {
            telemetry.addData("broke", 0);
        }
        telemetry.update();
        //driveWay(-0.6,0.6,0.6,-0.6, 1000);
        //driveWay(0.4,-0.4,-0.4,0.4, 200);
        // fL.setPower(0);
        // fR.setPower(0);
        // bL.setPower(0);
        // bR.setPower(0);
    }
    public void driveWay(double lF, double rF, double lB, double rB, int s){
        runtime.reset();
        while(opModeIsActive() && (runtime.milliseconds() < s)){
            // fL.setPower(lF);
            // fR.setPower(rF);
            // bL.setPower(lB);
            // bR.setPower(rB);
        }
        // fL.setPower(0);
        // fR.setPower(0);
        // bL.setPower(0);
        // bR.setPower(0);
    }
}
