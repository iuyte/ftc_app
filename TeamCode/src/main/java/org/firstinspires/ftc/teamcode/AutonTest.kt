package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation

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

        robot.motors[1]!!.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

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

        robot.motors.forEach { motor ->
            motor!!.power = 0.7
        }

        robot.motors[0]!!.targetPosition = 7750
        robot.motors[2]!!.targetPosition = 9200
        robot.motors[3]!!.targetPosition = 7750

        for (i in 0..robot.motors.size) {
            var it = robot.motors[i]!!
            if (i == 1) {
                break
            }

            while (it.isBusy && it.currentPosition < it.targetPosition && opModeIsActive()) {
                angle = imu!!.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)!!.firstAngle
                for (m in 0..robot.motors.size) {
                    if (m == 1) {
                        break
                    }
                    var motor = robot.motors[m]!!
                    motor.targetPosition = if (motor.isBusy) motor.targetPosition else motor.currentPosition + 1
                }

                if (angle > 0) {
                    robot.motors[0]!!.power = .7 - (angle / 7)
                    robot.motors[2]!!.power = .7 - (angle / 7)
                } else if (angle < 0) {
                    robot.motors[1]!!.power = .7 + (angle / 7)
                    robot.motors[3]!!.power = .7 + (angle / 7)
                } else {
                    robot.motors.forEach { motor ->
                        motor!!.power = .7
                    }
                }

                telemetry.addData("Left", robot.motors[0]!!.currentPosition)
                telemetry.addData("Right", robot.motors[1]!!.currentPosition)
                telemetry.addData("Angle", angle)
                telemetry.update()
                robot.waitForTick(25)
            }
        }
    }
}