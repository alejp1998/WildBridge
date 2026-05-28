package dji.sampleV5.aircraft.edge

import android.content.Context
import android.media.Image
import android.net.Uri
import dji.v5.ux.detection.DetectedTarget
import org.tensorflow.lite.DataType
import org.tensorflow.lite.Interpreter
import java.io.Closeable
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.max
import kotlin.math.min

class YoloTfliteDetector(
    private val modelBuffer: ByteBuffer,
    private val labels: List<String> = listOf("person"),
    private val confidenceThreshold: Float = 0.25f
) : Closeable {

    companion object {
        private const val CHANNELS = 3

        fun fromUri(
            context: Context,
            modelUri: Uri,
            labels: List<String> = listOf("person"),
            confidenceThreshold: Float = 0.25f
        ): YoloTfliteDetector {
            return YoloTfliteDetector(loadModelBuffer(context, modelUri), labels, confidenceThreshold)
        }

        private fun loadModelBuffer(context: Context, modelUri: Uri): ByteBuffer {
            val bytes = context.contentResolver.openInputStream(modelUri)?.use { it.readBytes() }
                ?: throw IllegalArgumentException("Could not open model: $modelUri")
            return ByteBuffer.allocateDirect(bytes.size).order(ByteOrder.nativeOrder()).apply {
                put(bytes)
                rewind()
            }
        }
    }

    private val interpreter = Interpreter(modelBuffer, Interpreter.Options().apply { setNumThreads(4) })
    private val inputTensor = interpreter.getInputTensor(0)
    private val outputTensor = interpreter.getOutputTensor(0)
    private val inputShape = inputTensor.shape()
    private val outputShape = outputTensor.shape()
    private val inputHeight = inputShape.getOrNull(1) ?: 320
    private val inputWidth = inputShape.getOrNull(2) ?: 320
    private val inputDataType = inputTensor.dataType()
    private val outputDataType = outputTensor.dataType()
    private val inputQuantization = inputTensor.quantizationParams()
    private val outputQuantization = outputTensor.quantizationParams()
    private val outputBoxes = outputShape.getOrNull(1) ?: 300
    private val outputValues = outputShape.getOrNull(2) ?: 6

    private val inputBuffer: ByteBuffer = ByteBuffer
        .allocateDirect(1 * inputWidth * inputHeight * CHANNELS * bytesPerElement(inputDataType))
        .order(ByteOrder.nativeOrder())
    private val outputBuffer: ByteBuffer = ByteBuffer
        .allocateDirect(outputShape.fold(1) { total, dim -> total * dim } * bytesPerElement(outputDataType))
        .order(ByteOrder.nativeOrder())

    fun detectNv21(frameData: ByteArray, offset: Int, length: Int, frameWidth: Int, frameHeight: Int): List<DetectedTarget> {
        if (frameWidth <= 0 || frameHeight <= 0 || length < frameWidth * frameHeight) return emptyList()

        val scale = min(inputWidth.toFloat() / frameWidth.toFloat(), inputHeight.toFloat() / frameHeight.toFloat())
        val resizedWidth = (frameWidth * scale).toInt().coerceAtLeast(1)
        val resizedHeight = (frameHeight * scale).toInt().coerceAtLeast(1)
        val padX = (inputWidth - resizedWidth) / 2
        val padY = (inputHeight - resizedHeight) / 2

        inputBuffer.rewind()
        for (modelY in 0 until inputHeight) {
            for (modelX in 0 until inputWidth) {
                if (modelX < padX || modelX >= padX + resizedWidth || modelY < padY || modelY >= padY + resizedHeight) {
                    putInputValue(114f / 255f)
                    putInputValue(114f / 255f)
                    putInputValue(114f / 255f)
                    continue
                }

                val sourceX = ((modelX - padX) / scale).toInt().coerceIn(0, frameWidth - 1)
                val sourceY = ((modelY - padY) / scale).toInt().coerceIn(0, frameHeight - 1)
                val rgb = nv21ToRgb(frameData, offset, length, frameWidth, frameHeight, sourceX, sourceY)
                putInputValue(rgb.red / 255f)
                putInputValue(rgb.green / 255f)
                putInputValue(rgb.blue / 255f)
            }
        }

        inputBuffer.rewind()
        outputBuffer.rewind()
        interpreter.run(inputBuffer, outputBuffer)
        outputBuffer.rewind()

        val targets = mutableListOf<DetectedTarget>()
        for (i in 0 until outputBoxes) {
            if (outputValues < 6) break
            val rowOffset = i * outputValues
            val score = getOutputValue(rowOffset + 4)
            if (score < confidenceThreshold) continue

            val classIndex = getOutputValue(rowOffset + 5).toInt().coerceAtLeast(0)
            val label = labels.getOrElse(classIndex) { "class_$classIndex" }.uppercase()
            val x1 = toInputCoordinate(getOutputValue(rowOffset))
            val y1 = toInputCoordinate(getOutputValue(rowOffset + 1))
            val x2 = toInputCoordinate(getOutputValue(rowOffset + 2))
            val y2 = toInputCoordinate(getOutputValue(rowOffset + 3))

            val left = ((x1 - padX) / scale).coerceIn(0f, frameWidth.toFloat()) / frameWidth.toFloat()
            val top = ((y1 - padY) / scale).coerceIn(0f, frameHeight.toFloat()) / frameHeight.toFloat()
            val right = ((x2 - padX) / scale).coerceIn(0f, frameWidth.toFloat()) / frameWidth.toFloat()
            val bottom = ((y2 - padY) / scale).coerceIn(0f, frameHeight.toFloat()) / frameHeight.toFloat()
            if (right <= left || bottom <= top) continue

            targets.add(
                DetectedTarget(
                    index = i,
                    type = "EDGE_$label",
                    left = left.toDouble(),
                    top = top.toDouble(),
                    right = right.toDouble(),
                    bottom = bottom.toDouble(),
                    confidence = score.toDouble()
                )
            )
        }
        return targets
    }

    fun detectYuv420(image: Image): List<DetectedTarget> {
        val frameWidth = image.width
        val frameHeight = image.height
        if (frameWidth <= 0 || frameHeight <= 0 || image.planes.size < 3) return emptyList()

        val scale = min(inputWidth.toFloat() / frameWidth.toFloat(), inputHeight.toFloat() / frameHeight.toFloat())
        val resizedWidth = (frameWidth * scale).toInt().coerceAtLeast(1)
        val resizedHeight = (frameHeight * scale).toInt().coerceAtLeast(1)
        val padX = (inputWidth - resizedWidth) / 2
        val padY = (inputHeight - resizedHeight) / 2

        val yPlane = image.planes[0]
        val uPlane = image.planes[1]
        val vPlane = image.planes[2]
        inputBuffer.rewind()
        for (modelY in 0 until inputHeight) {
            for (modelX in 0 until inputWidth) {
                if (modelX < padX || modelX >= padX + resizedWidth || modelY < padY || modelY >= padY + resizedHeight) {
                    putInputValue(114f / 255f)
                    putInputValue(114f / 255f)
                    putInputValue(114f / 255f)
                    continue
                }

                val sourceX = ((modelX - padX) / scale).toInt().coerceIn(0, frameWidth - 1)
                val sourceY = ((modelY - padY) / scale).toInt().coerceIn(0, frameHeight - 1)
                val rgb = yuv420ToRgb(yPlane, uPlane, vPlane, sourceX, sourceY)
                putInputValue(rgb.red / 255f)
                putInputValue(rgb.green / 255f)
                putInputValue(rgb.blue / 255f)
            }
        }

        inputBuffer.rewind()
        outputBuffer.rewind()
        interpreter.run(inputBuffer, outputBuffer)
        outputBuffer.rewind()

        return collectTargets(frameWidth, frameHeight, scale, padX, padY)
    }

    override fun close() {
        interpreter.close()
    }

    private fun toInputCoordinate(value: Float): Float {
        return if (value <= 1.5f) value * inputWidth else value
    }

    private fun putInputValue(value: Float) {
        when (inputDataType) {
            DataType.FLOAT32 -> inputBuffer.putFloat(value)
            DataType.UINT8, DataType.INT8 -> inputBuffer.put(quantize(value, inputDataType, inputQuantization.scale, inputQuantization.zeroPoint))
            else -> throw IllegalArgumentException("Unsupported input tensor type: $inputDataType")
        }
    }

    private fun getOutputValue(index: Int): Float {
        val byteIndex = index * bytesPerElement(outputDataType)
        return when (outputDataType) {
            DataType.FLOAT32 -> outputBuffer.getFloat(byteIndex)
            DataType.UINT8 -> dequantize(outputBuffer.get(byteIndex).toInt() and 0xFF, outputQuantization.scale, outputQuantization.zeroPoint)
            DataType.INT8 -> dequantize(outputBuffer.get(byteIndex).toInt(), outputQuantization.scale, outputQuantization.zeroPoint)
            else -> throw IllegalArgumentException("Unsupported output tensor type: $outputDataType")
        }
    }

    private fun quantize(value: Float, dataType: DataType, scale: Float, zeroPoint: Int): Byte {
        if (scale == 0f) return 0
        val quantized = kotlin.math.round(value / scale + zeroPoint).toInt()
        return when (dataType) {
            DataType.UINT8 -> quantized.coerceIn(0, 255).toByte()
            DataType.INT8 -> quantized.coerceIn(-128, 127).toByte()
            else -> 0
        }
    }

    private fun dequantize(value: Int, scale: Float, zeroPoint: Int): Float {
        return if (scale == 0f) value.toFloat() else (value - zeroPoint) * scale
    }

    private fun bytesPerElement(dataType: DataType): Int {
        return when (dataType) {
            DataType.FLOAT32 -> java.lang.Float.BYTES
            DataType.UINT8, DataType.INT8 -> java.lang.Byte.BYTES
            else -> throw IllegalArgumentException("Unsupported tensor type: $dataType")
        }
    }

    private data class Rgb(val red: Float, val green: Float, val blue: Float)

    private fun collectTargets(frameWidth: Int, frameHeight: Int, scale: Float, padX: Int, padY: Int): List<DetectedTarget> {
        val targets = mutableListOf<DetectedTarget>()
        for (i in 0 until outputBoxes) {
            if (outputValues < 6) break
            val rowOffset = i * outputValues
            val score = getOutputValue(rowOffset + 4)
            if (score < confidenceThreshold) continue

            val classIndex = getOutputValue(rowOffset + 5).toInt().coerceAtLeast(0)
            val label = labels.getOrElse(classIndex) { "class_$classIndex" }.uppercase()
            val x1 = toInputCoordinate(getOutputValue(rowOffset))
            val y1 = toInputCoordinate(getOutputValue(rowOffset + 1))
            val x2 = toInputCoordinate(getOutputValue(rowOffset + 2))
            val y2 = toInputCoordinate(getOutputValue(rowOffset + 3))

            val left = ((x1 - padX) / scale).coerceIn(0f, frameWidth.toFloat()) / frameWidth.toFloat()
            val top = ((y1 - padY) / scale).coerceIn(0f, frameHeight.toFloat()) / frameHeight.toFloat()
            val right = ((x2 - padX) / scale).coerceIn(0f, frameWidth.toFloat()) / frameWidth.toFloat()
            val bottom = ((y2 - padY) / scale).coerceIn(0f, frameHeight.toFloat()) / frameHeight.toFloat()
            if (right <= left || bottom <= top) continue

            targets.add(
                DetectedTarget(
                    index = i,
                    type = "EDGE_$label",
                    left = left.toDouble(),
                    top = top.toDouble(),
                    right = right.toDouble(),
                    bottom = bottom.toDouble(),
                    confidence = score.toDouble()
                )
            )
        }
        return targets
    }

    private fun yuv420ToRgb(yPlane: Image.Plane, uPlane: Image.Plane, vPlane: Image.Plane, x: Int, y: Int): Rgb {
        val yValue = planeValue(yPlane, x, y)
        val uValue = planeValue(uPlane, x / 2, y / 2) - 128
        val vValue = planeValue(vPlane, x / 2, y / 2) - 128
        return yuvToRgb(yValue, uValue, vValue)
    }

    private fun planeValue(plane: Image.Plane, x: Int, y: Int): Int {
        val index = y * plane.rowStride + x * plane.pixelStride
        val buffer = plane.buffer
        return if (index in 0 until buffer.limit()) buffer.get(index).toInt() and 0xFF else 114
    }

    private fun nv21ToRgb(data: ByteArray, offset: Int, length: Int, width: Int, height: Int, x: Int, y: Int): Rgb {
        val frameSize = width * height
        val yIndex = offset + y * width + x
        val uvIndex = offset + frameSize + (y / 2) * width + (x and 1.inv())
        if (yIndex >= offset + length || uvIndex + 1 >= offset + length) return Rgb(114f, 114f, 114f)

        val yValue = data[yIndex].toInt() and 0xFF
        val vValue = (data[uvIndex].toInt() and 0xFF) - 128
        val uValue = (data[uvIndex + 1].toInt() and 0xFF) - 128
        return yuvToRgb(yValue, uValue, vValue)
    }

    private fun yuvToRgb(yValue: Int, uValue: Int, vValue: Int): Rgb {
        val c = max(0, yValue - 16)
        val red = ((298 * c + 409 * vValue + 128) shr 8).coerceIn(0, 255)
        val green = ((298 * c - 100 * uValue - 208 * vValue + 128) shr 8).coerceIn(0, 255)
        val blue = ((298 * c + 516 * uValue + 128) shr 8).coerceIn(0, 255)
        return Rgb(red.toFloat(), green.toFloat(), blue.toFloat())
    }

}