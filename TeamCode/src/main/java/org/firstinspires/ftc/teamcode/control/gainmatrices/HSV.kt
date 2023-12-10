package org.firstinspires.ftc.teamcode.control.gainmatrices

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
}
