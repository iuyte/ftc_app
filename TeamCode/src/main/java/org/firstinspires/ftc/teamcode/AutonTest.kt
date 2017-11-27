package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

/**
 * Created by ethan on 7/29/17.
 */

@Autonomous(name = "Test Auton", group = "Robot")

class AutonTest : LinearOpMode() {

    /* Declare OpMode members. */
    private var robot = Hardware(this, Hardware.DriveMode.Holonomic, true) // use the class created to define a robot's hardware
    private var imu: BNO055IMU? = null
    private var angle : Float = 0f

    override fun runOpMode() {
        /*
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, DcMotor.RunMode.RUN_TO_POSITION)

        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.loggingEnabled = true
        parameters.useExternalCrystal = true
        parameters.mode = BNO055IMU.SensorMode.IMU
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu!!.initialize(parameters)

        waitForStart()

        robot.motors[0]!!.targetPosition = 7500
        robot.motors[1]!!.targetPosition = 7500
        robot.motors[2]!!.targetPosition = 7500
        robot.motors[3]!!.targetPosition = 7500

        robot.motors.forEach { motor ->
            motor!!.power = 0.7
        }

        robot.motors.forEach { motor ->
            val it = motor!!

            while (it.isBusy && it.currentPosition < it.targetPosition && opModeIsActive()) {
                angle = imu!!.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)!!.firstAngle
                robot.motors.forEach { m ->
                    m!!.targetPosition = if (m.isBusy) m.targetPosition else m.currentPosition + 1
                }

                when {
                    angle > 0 -> {
                        robot.motors[0]!!.power = .7 - (angle / 5)
                        robot.motors[2]!!.power = .7 - (angle / 5)
                    }
                    angle < 0 -> {
                        robot.motors[1]!!.power = .7 + (angle / 5)
                        robot.motors[3]!!.power = .7 + (angle / 5)
                    }
                    else -> {
                        robot.motors.forEach { m ->
                            m!!.power = .7
                        }
                    }
                }

                robot.motors.forEach {
                    telemetry.addData(it!!.deviceName, "%d", it.currentPosition)
                }
                telemetry.addData("Angle", angle)
                telemetry.update()
                robot.waitForTick(25)
            }
        }
    }
}