package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

/**
 * Created by ethan on 11/4/2017.
 */

@Autonomous(name="VuMark Id", group ="VuMarks")
class VuMarkIdent : LinearOpMode() {

    private var robot = Hardware(hardwareMap, this, Hardware.DriveMode.Holonomic, false, true, false) // use the class created to define a robot's hardware

    override fun runOpMode() {
        robot.init(hardwareMap)
        telemetry.addData(">", "Press Play to start")
        telemetry.update()
        waitForStart()

        robot.relicTrackables!!.activate()

        while (opModeIsActive()) {
            var vuMark = robot.vuMark()
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark)
            } else {
                telemetry.addData("VuMark", "not visible")
            }

            telemetry.update()
        }
    }
}
