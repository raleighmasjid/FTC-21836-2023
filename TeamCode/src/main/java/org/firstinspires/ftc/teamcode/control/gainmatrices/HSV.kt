package org.firstinspires.ftc.teamcode.control.gainmatrices

import org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry

data class HSV @JvmOverloads constructor(
    @JvmField var hue: Double = 0.0,
    @JvmField var saturation: Double = 0.0,
    @JvmField var value: Double = 0.0,
) {
    fun inRange(min:HSV, max:HSV): Boolean {

        val hueInRange = (min.hue <= hue) && (hue <= max.hue)
        val saturationInRange = (min.saturation <= saturation) && (saturation <= max.saturation)
        val valueInRange = (min.value <= value) && (value <= max.value)

        return hueInRange && saturationInRange && valueInRange
    }

    fun toTelemetry(title: String) {
        mTelemetry.addLine("$title:")
        mTelemetry.addData("Hue", hue)
        mTelemetry.addData("Saturation", saturation)
        mTelemetry.addData("Value", value)
    }
}
