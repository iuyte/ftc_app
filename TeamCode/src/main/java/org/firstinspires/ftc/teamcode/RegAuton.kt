package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.*

/**
 * Created by ethan on 7/29/17.
 */

@Autonomous(name = "Regular Auton", group = "Robot")

class   RegAuton : LinearOpMode() {

    /* Declare OpMode members. */
    private var robot = Hardware(hardwareMap, this, Hardware.DriveMode.Holonomic, true, true) // use the class created to define a robot's hardware

    override fun runOpMode() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, DcMotor.RunMode.RUN_TO_POSITION)
        waitForStart()
        robot.relicTrackables!!.activate()

        var vuMark = robot.vuMark()
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {
            vuMark = robot.vuMark()
            robot.waitForTick(25)
            telemetry.addData("VuMark", "None")
            telemetry.update()
            robot.waitForTick(25)
        }
        telemetry.addData("VuMark", "%s", vuMark)

        robot.motors["drive"]!![0].targetPosition = 3500
        robot.motors["drive"]!![1].targetPosition = -8500
        robot.motors["drive"]!![2].targetPosition = 10408
        robot.motors["drive"]!![3].targetPosition = -8406

        robot.motors["drive"]!!.forEach {
            while (it.isBusy && it.currentPosition < it.targetPosition && opModeIsActive()) {
                robot.motors["drive"]!!.forEach { motor ->
                    motor.targetPosition = if (motor.isBusy) motor.targetPosition else motor.currentPosition + 1
                }
                robot.motors["drive"]!!.forEach { motor ->
                    telemetry.addData(motor.deviceName, "%d", motor.currentPosition)
                }
                telemetry.update()
                robot.waitForTick(25)
            }
        }
    }
}