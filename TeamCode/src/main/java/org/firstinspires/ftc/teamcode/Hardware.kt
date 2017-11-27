package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

/**
 * Created by ethan on 7/29/17.
 */

/* Makes moving with the robot easier */
class Hardware(private var mode : OpMode,
               private var driveMode : DriveMode,
               private var useSensors : Boolean = false,
               private var useVuforia : Boolean = false,
               private var useMotors : Boolean = true) {
    // Color sensor
    var colorSensor: ColorSensor? = null
    var button: DigitalChannel? = null
    var revButton: DigitalChannel? = null

    // Vuforia members
    var vuforia: VuforiaLocalizer? = null
    private var cameraMonitorViewId : Int? = null
    private var vuforiaParams : VuforiaLocalizer.Parameters? = null
    private val vuforiaKey =
            "AQ+eJzP/////AAAAGQ6Eyl3tQ0fqvcl2vinKWkR8t2wTESOSuHo32BhQvWxRBBxt+4C2BtrXTh" +
                    "Ygcyd9fzaZ1tTjW8viQzNJAVYyL50GIG+hKJb6mHRq/iYJ53ve+oETR1t5ZBaVmq2l" +
                    "qljNaxyRJURYSLp52UoNi2RpvW0xvVIZ4KBoClIUI4KRkiozACNg0GsxDIDnAiBx5i" +
                    "iPfug5PklSyPnKHkhKf+mtRhoY8HcittFKh4lQkI5LtJOm6lH/K6CCa7RzVUkRbI9m" +
                    "bxVybXiOLtpN/Yr+FDS0bM19czvflS+URyrO2J5APOipQ1XZ1wieP/wp75NFXkV0V3" +
                    "hWcVAkWHQr9aJhZtkYUGjrb94QP7OOF7h6Ftl+ls2j"

    // VuMark specific
    var relicTrackables : VuforiaTrackables? = null
    var relicTemplate : VuforiaTrackable? = null

    enum class COLORS {
        RED,
        BLUE,
        NONE,
    }

    /* The motors on the drive */
    var motors: Map<String, Array<DcMotor?>> = emptyMap()

    /* private OpMode members. */
    private var hwMap: HardwareMap? = null
    private val period = ElapsedTime()

    /* The different drive modes */
    enum class DriveMode {
        Tank,
        Holonomic,
        Mecanum,
    }

    /* Initialize standard Hardware interfaces */
    fun init(ahwMap : HardwareMap, motorMode : DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        // Save reference to Hardware map
        hwMap = ahwMap

        if (useSensors) {
            // Initialize sensors
            colorSensor = hwMap!!.get(ColorSensor::class.java, "rev color")
            button = hwMap!!.get(DigitalChannel::class.java, "button")
            revButton = hwMap!!.get(DigitalChannel::class.java, "rbutton")
        }

        if (useMotors) {
            // Define and Initialize Motors
            motors = arrayOf(
                    hwMap!!.dcMotor.get("front left drive"),
                    hwMap!!.dcMotor.get("front right drive"),
                    hwMap!!.dcMotor.get("back left drive"),
                    hwMap!!.dcMotor.get("back right drive"))

            // Set all motors to zero power and to run without encoders
            motors.forEach {
                it!!.power = 0.0
                it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }

            Thread.sleep(50)
            motors.forEach {
                it!!.mode = motorMode
            }

            motors[0]!!.direction = DcMotorSimple.Direction.REVERSE
            motors[1]!!.direction = DcMotorSimple.Direction.FORWARD
            motors[2]!!.direction = DcMotorSimple.Direction.REVERSE
            motors[3]!!.direction = DcMotorSimple.Direction.FORWARD
        }

        if (useVuforia) {
            // Vuforia initialization, setup
            cameraMonitorViewId = hwMap!!.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hwMap!!.appContext.packageName)
            vuforiaParams = VuforiaLocalizer.Parameters(cameraMonitorViewId!!)
            vuforiaParams!!.vuforiaLicenseKey = vuforiaKey
            vuforiaParams!!.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
            this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams)

            relicTrackables = this.vuforia!!.loadTrackablesFromAsset("RelicVuMark")
            relicTemplate = relicTrackables!![0]
            relicTemplate!!.name = "relicVuMarkTemplate"
        }

        mode.telemetry.addData("Robot", "Initialized!")
    }

    fun setDrive(upLeft: Float, upright: Float, downLeft: Float, downright: Float) {
        motors[0]!!.power = upLeft.toDouble()
        motors[1]!!.power = upright.toDouble()
        motors[2]!!.power = downLeft.toDouble()
        motors[3]!!.power = downright.toDouble()

        // Send telemetry message to signify robot running;
        motors.forEach {
            mode.telemetry.addData(it!!.deviceName, "%d", it.currentPosition)
        }
    }

    private fun holonomic(x : Float, y : Float, θ : Float) {
        val fl = -x + y - θ
        val fr =  x + y + θ
        val bl =  x + y - θ
        val br = -x + y + θ
        setDrive(fl, fr, bl, br)
    }

    private fun mecanum(x : Float, y : Float, θ : Float) {
        val fl = x + y + θ
        val fr = x - y + θ
        val bl = x - y + θ
        val br = x - y - θ
        setDrive(fl, fr, bl, br)
    }

    fun drive(x : Float, y : Float, θ : Float) {
        when (driveMode) {
            DriveMode.Holonomic -> holonomic(x, y, θ)
            DriveMode.Mecanum -> mecanum(x, y, θ)
            else -> mode.telemetry.addData("Error", "Called Hardware::Drive, current driveMode not found")
        }
    }

    fun color(c : ColorSensor? = colorSensor) : COLORS {
        val rgb = arrayOf(
                c!!.red(),
                c.blue(),
                c.green())
        if (rgb[0] > rgb[2] + 25) {
            return COLORS.RED
        } else if (rgb[2] > rgb[0] + 15) {
            return COLORS.BLUE
        }
        return COLORS.NONE
    }

    fun vuMark() : RelicRecoveryVuMark = RelicRecoveryVuMark.from(relicTemplate)

    /***

     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.

     * @param periodMs Length of wait cycle in mSec.
     */
    fun waitForTick(periodMs: Long) {
        val remaining = periodMs - period.milliseconds().toLong()

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset()
    }
}
