package org.firstinspires.ftc.teamcode.control.gainmatrices

data class HSV @JvmOverloads constructor(
    @JvmField var hue: Float = 0.0F,
    @JvmField var saturation: Float = 0.0F,
    @JvmField var value: Float = 0.0F,
) {
    fun inRange(min:HSV, max:HSV): Boolean {

        val hueInRange = (min.hue <= hue) && (hue <= max.hue)
        val saturationInRange = (min.saturation <= saturation) && (saturation <= max.saturation)
        val valueInRange = (min.value <= value) && (value <= max.value)

        return hueInRange && saturationInRange && valueInRange
    }
}
