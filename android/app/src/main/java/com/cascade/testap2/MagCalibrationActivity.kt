package com.cascade.testap2

import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat

/**
 * Magnetometer Calibration Activity
 *
 * Displays and manages magnetometer calibration data:
 * - Hard-iron offsets (3 floats)
 * - Soft-iron correction matrix (3x3)
 *
 * Provides test data loading for development/testing.
 */
class MagCalibrationActivity : AppCompatActivity() {

    private lateinit var bleManager: AutopilotBleManager

    // UI elements
    private lateinit var statusIndicator: View
    private lateinit var statusText: TextView
    private lateinit var hardIronX: TextView
    private lateinit var hardIronY: TextView
    private lateinit var hardIronZ: TextView
    private lateinit var soft00: TextView
    private lateinit var soft01: TextView
    private lateinit var soft02: TextView
    private lateinit var soft10: TextView
    private lateinit var soft11: TextView
    private lateinit var soft12: TextView
    private lateinit var soft20: TextView
    private lateinit var soft21: TextView
    private lateinit var soft22: TextView
    private lateinit var matrixStatus: TextView

    private var currentMagCal: MagCalData? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_mag_calibration)

        // Get shared BLE manager from application
        bleManager = (application as CascadeApplication).bleManager

        initViews()
        setupCallbacks()
        updateConnectionStatus()

        // Read current calibration on start
        if (bleManager.isConnected()) {
            bleManager.readMagCal()
        }
    }

    private fun initViews() {
        statusIndicator = findViewById(R.id.statusIndicator)
        statusText = findViewById(R.id.statusText)
        hardIronX = findViewById(R.id.hardIronX)
        hardIronY = findViewById(R.id.hardIronY)
        hardIronZ = findViewById(R.id.hardIronZ)
        soft00 = findViewById(R.id.soft00)
        soft01 = findViewById(R.id.soft01)
        soft02 = findViewById(R.id.soft02)
        soft10 = findViewById(R.id.soft10)
        soft11 = findViewById(R.id.soft11)
        soft12 = findViewById(R.id.soft12)
        soft20 = findViewById(R.id.soft20)
        soft21 = findViewById(R.id.soft21)
        soft22 = findViewById(R.id.soft22)
        matrixStatus = findViewById(R.id.matrixStatus)

        // Action buttons
        findViewById<Button>(R.id.btnRead).setOnClickListener { onReadClicked() }
        findViewById<Button>(R.id.btnSave).setOnClickListener { onSaveClicked() }
        findViewById<Button>(R.id.btnReset).setOnClickListener { onResetClicked() }
        findViewById<Button>(R.id.btnClose).setOnClickListener { finish() }

        // Test data buttons
        findViewById<Button>(R.id.btnTest90).setOnClickListener { onTest90Clicked() }
        findViewById<Button>(R.id.btnTest180).setOnClickListener { onTest180Clicked() }
        findViewById<Button>(R.id.btnClearTest).setOnClickListener { onClearTestClicked() }
    }

    private fun setupCallbacks() {
        bleManager.onMagCalReceived = { magCal ->
            runOnUiThread {
                currentMagCal = magCal
                updateDisplay(magCal)
                showToast("Calibration data received")
            }
        }

        bleManager.onConnectionStateChanged = { connected ->
            runOnUiThread {
                updateConnectionStatus()
                if (connected) {
                    bleManager.readMagCal()
                }
            }
        }
    }

    private fun updateConnectionStatus() {
        val connected = bleManager.isConnected()
        val demoMode = bleManager.isDemoMode()

        when {
            demoMode -> {
                statusIndicator.backgroundTintList = ContextCompat.getColorStateList(this, android.R.color.holo_orange_light)
                statusText.text = "Demo Mode"
            }
            connected -> {
                statusIndicator.backgroundTintList = ContextCompat.getColorStateList(this, android.R.color.holo_green_light)
                statusText.text = "Connected"
            }
            else -> {
                statusIndicator.backgroundTintList = ContextCompat.getColorStateList(this, android.R.color.darker_gray)
                statusText.text = "Not connected"
            }
        }
    }

    private fun updateDisplay(magCal: MagCalData) {
        // Hard-iron offsets
        hardIronX.text = String.format("%.2f", magCal.hardIronX)
        hardIronY.text = String.format("%.2f", magCal.hardIronY)
        hardIronZ.text = String.format("%.2f", magCal.hardIronZ)

        // Soft-iron matrix
        val s = magCal.softIron
        soft00.text = String.format("%.3f", s[0])
        soft01.text = String.format("%.3f", s[1])
        soft02.text = String.format("%.3f", s[2])
        soft10.text = String.format("%.3f", s[3])
        soft11.text = String.format("%.3f", s[4])
        soft12.text = String.format("%.3f", s[5])
        soft20.text = String.format("%.3f", s[6])
        soft21.text = String.format("%.3f", s[7])
        soft22.text = String.format("%.3f", s[8])

        // Matrix status
        matrixStatus.text = if (magCal.isIdentity()) {
            "Identity matrix (no correction)"
        } else {
            "Custom calibration applied"
        }
    }

    private fun onReadClicked() {
        if (bleManager.readMagCal()) {
            showToast("Reading calibration...")
        } else {
            showToast("Failed to read - not connected")
        }
    }

    private fun onSaveClicked() {
        if (bleManager.saveMagCal()) {
            showToast("Saving to NVS...")
        } else {
            showToast("Failed to save - not connected")
        }
    }

    private fun onResetClicked() {
        if (bleManager.resetMagCal()) {
            showToast("Resetting to defaults...")
            // Read back the reset values
            bleManager.readMagCal()
        } else {
            showToast("Failed to reset - not connected")
        }
    }

    private fun onTest90Clicked() {
        // 90-degree test data: rotation around Z axis
        val testCal = MagCalData(
            hardIronX = 15.5f,
            hardIronY = -8.2f,
            hardIronZ = 42.0f,
            softIron = floatArrayOf(
                0.0f, 1.0f, 0.0f,   // Swapped X and Y
                -1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            )
        )

        if (bleManager.setMagCal(testCal)) {
            showToast("90° test data loaded")
            updateDisplay(testCal)
            currentMagCal = testCal
        } else {
            showToast("Failed - not connected")
        }
    }

    private fun onTest180Clicked() {
        // 180-degree test data: inverted X and Y
        val testCal = MagCalData(
            hardIronX = -22.3f,
            hardIronY = 31.7f,
            hardIronZ = -5.5f,
            softIron = floatArrayOf(
                -1.0f, 0.0f, 0.0f,   // Inverted X
                0.0f, -1.0f, 0.0f,   // Inverted Y
                0.0f, 0.0f, 1.0f
            )
        )

        if (bleManager.setMagCal(testCal)) {
            showToast("180° test data loaded")
            updateDisplay(testCal)
            currentMagCal = testCal
        } else {
            showToast("Failed - not connected")
        }
    }

    private fun onClearTestClicked() {
        // Reset to identity/zero calibration
        val defaultCal = MagCalData()

        if (bleManager.setMagCal(defaultCal)) {
            showToast("Test data cleared")
            updateDisplay(defaultCal)
            currentMagCal = defaultCal
        } else {
            showToast("Failed - not connected")
        }
    }

    private fun showToast(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    override fun onDestroy() {
        super.onDestroy()
        // Don't clear callbacks - they're shared with main activity
    }
}
