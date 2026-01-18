package com.cascade.testap2

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.cascade.testap2.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var bleManager: AutopilotBleManager

    private var currentStatus = AutopilotStatus()
    private var currentHeading = 0f
    private var targetHeading = 0f
    private var rudderAngle = 0f

    companion object {
        private const val PERMISSION_REQUEST_CODE = 100
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        bleManager = (application as CascadeApplication).bleManager
        setupBleCallbacks()
        setupClickListeners()
        updateUI()

        if (checkPermissions()) {
            // Ready to scan
        } else {
            requestPermissions()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        // BLE manager is now managed by CascadeApplication, no need to close here
    }

    // =========================================================================
    // PERMISSIONS
    // =========================================================================

    private fun checkPermissions(): Boolean {
        val permissions = getRequiredPermissions()
        return permissions.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun getRequiredPermissions(): Array<String> {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }
    }

    private fun requestPermissions() {
        ActivityCompat.requestPermissions(this, getRequiredPermissions(), PERMISSION_REQUEST_CODE)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST_CODE) {
            if (grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
                Toast.makeText(this, "Permissions granted", Toast.LENGTH_SHORT).show()
            } else {
                Toast.makeText(this, "Bluetooth permissions required", Toast.LENGTH_LONG).show()
            }
        }
    }

    // =========================================================================
    // BLE CALLBACKS
    // =========================================================================

    private fun setupBleCallbacks() {
        bleManager.onConnectionStateChanged = { connected ->
            runOnUiThread {
                if (connected) {
                    if (bleManager.isDemoMode()) {
                        binding.btnConnect.text = getString(R.string.disconnect)
                        binding.statusConnection.text = getString(R.string.demo_mode_active)
                        binding.statusConnection.setTextColor(getColor(R.color.demo_mode))
                        binding.btnDemo.text = "Exit Demo"
                    } else {
                        binding.btnConnect.text = getString(R.string.disconnect)
                        binding.statusConnection.text = getString(R.string.connected)
                        binding.statusConnection.setTextColor(getColor(R.color.connected))
                    }
                } else {
                    binding.btnConnect.text = getString(R.string.connect)
                    binding.statusConnection.text = getString(R.string.disconnected)
                    binding.statusConnection.setTextColor(getColor(R.color.disconnected))
                    binding.btnDemo.text = getString(R.string.demo_mode)
                }
                updateUI()
            }
        }

        bleManager.onStatusReceived = { status ->
            runOnUiThread {
                currentStatus = status
                updateStatusDisplay()
            }
        }

        bleManager.onHeadingReceived = { current, target ->
            runOnUiThread {
                currentHeading = current
                targetHeading = target
                updateHeadingDisplay()
            }
        }

        bleManager.onRudderReceived = { angle, motorStatus ->
            runOnUiThread {
                rudderAngle = angle
                updateRudderDisplay()
            }
        }

        bleManager.onDeviceFound = { device ->
            runOnUiThread {
                showConnectDialog(device)
            }
        }

        bleManager.onScanStopped = {
            runOnUiThread {
                binding.btnConnect.isEnabled = true
                binding.btnConnect.text = getString(R.string.connect)
                Toast.makeText(this, "Device not found", Toast.LENGTH_SHORT).show()
            }
        }

        bleManager.onError = { error ->
            runOnUiThread {
                Toast.makeText(this, error, Toast.LENGTH_SHORT).show()
            }
        }
    }

    // =========================================================================
    // CLICK LISTENERS
    // =========================================================================

    private fun setupClickListeners() {
        // Demo Mode
        binding.btnDemo.setOnClickListener {
            if (bleManager.isDemoMode()) {
                bleManager.disconnect()
            } else if (!bleManager.isConnected()) {
                bleManager.enterDemoMode()
            }
        }

        // Connect/Disconnect
        binding.btnConnect.setOnClickListener {
            if (bleManager.isConnected()) {
                bleManager.disconnect()
            } else {
                binding.btnConnect.text = getString(R.string.scanning)
                binding.btnConnect.isEnabled = false
                bleManager.startScan()
            }
        }

        // Engage/Disengage
        binding.btnEngage.setOnClickListener {
            if (currentStatus.state == AutopilotState.ENGAGED) {
                bleManager.disengage()
            } else {
                bleManager.engage()
            }
        }

        // Heading adjustments
        binding.btnHeadingMinus10.setOnClickListener { bleManager.adjustHeading(-10) }
        binding.btnHeadingMinus1.setOnClickListener { bleManager.adjustHeading(-1) }
        binding.btnHeadingPlus1.setOnClickListener { bleManager.adjustHeading(1) }
        binding.btnHeadingPlus10.setOnClickListener { bleManager.adjustHeading(10) }

        // Calibration
        binding.btnCalibration.setOnClickListener {
            if (currentStatus.state == AutopilotState.CALIBRATION) {
                showCalibrationDialog()
            } else if (currentStatus.state == AutopilotState.IDLE) {
                bleManager.enterCalibration()
            }
        }

        // Parameters
        binding.btnParameters.setOnClickListener {
            val intent = Intent(this, ParametersActivity::class.java)
            startActivity(intent)
        }

        // Mag Calibration
        binding.btnMagCal.setOnClickListener {
            val intent = Intent(this, MagCalibrationActivity::class.java)
            startActivity(intent)
        }

        // Clear fault
        binding.btnClearFault.setOnClickListener {
            bleManager.clearFault()
        }
    }

    // =========================================================================
    // UI UPDATES
    // =========================================================================

    private fun updateUI() {
        val connected = bleManager.isConnected()

        // Enable/disable controls based on connection
        binding.btnEngage.isEnabled = connected
        binding.btnHeadingMinus10.isEnabled = connected
        binding.btnHeadingMinus1.isEnabled = connected
        binding.btnHeadingPlus1.isEnabled = connected
        binding.btnHeadingPlus10.isEnabled = connected
        binding.btnCalibration.isEnabled = connected
        binding.btnParameters.isEnabled = connected
        binding.btnMagCal.isEnabled = connected
        binding.btnClearFault.isEnabled = connected

        // Demo button state
        binding.btnDemo.isEnabled = !bleManager.isConnected() || bleManager.isDemoMode()
    }

    private fun updateStatusDisplay() {
        // State
        binding.statusState.text = currentStatus.state.name
        val stateColor = when (currentStatus.state) {
            AutopilotState.ENGAGED -> R.color.engaged
            AutopilotState.FAULTED -> R.color.faulted
            AutopilotState.CALIBRATION -> R.color.calibration
            else -> R.color.idle
        }
        binding.statusState.setTextColor(getColor(stateColor))

        // Engage button text
        binding.btnEngage.text = if (currentStatus.state == AutopilotState.ENGAGED) {
            getString(R.string.disengage)
        } else {
            getString(R.string.engage)
        }
        binding.btnEngage.isEnabled = bleManager.isConnected() &&
            (currentStatus.state == AutopilotState.IDLE || currentStatus.state == AutopilotState.ENGAGED)

        // Calibration button
        binding.btnCalibration.text = if (currentStatus.state == AutopilotState.CALIBRATION) {
            "CALIBRATING..."
        } else {
            getString(R.string.calibrate)
        }
        binding.btnCalibration.isEnabled = bleManager.isConnected() &&
            (currentStatus.state == AutopilotState.IDLE || currentStatus.state == AutopilotState.CALIBRATION)

        // Fault display
        if (currentStatus.state == AutopilotState.FAULTED) {
            binding.faultContainer.visibility = View.VISIBLE
            binding.faultText.text = "FAULT: ${currentStatus.fault.name}"
            binding.btnClearFault.visibility = View.VISIBLE
        } else {
            binding.faultContainer.visibility = View.GONE
            binding.btnClearFault.visibility = View.GONE
        }

        // Calibration status
        binding.statusCalibration.text = if (currentStatus.calibrationValid) "Cal: OK" else "Cal: NONE"
        binding.statusCalibration.setTextColor(
            getColor(if (currentStatus.calibrationValid) R.color.connected else R.color.disconnected)
        )
    }

    private fun updateHeadingDisplay() {
        binding.headingCurrent.text = String.format("%.1f°", currentHeading)
        binding.headingTarget.text = String.format("%.1f°", targetHeading)

        val error = normalizeAngle(targetHeading - currentHeading)
        binding.headingError.text = String.format("%+.1f°", error)
    }

    private fun updateRudderDisplay() {
        binding.rudderAngle.text = String.format("%+.1f°", rudderAngle)
        updateRudderIndicator(rudderAngle)
    }

    private fun updateRudderIndicator(angle: Float) {
        // Animate rudder indicator position (-35 to +35 degrees mapped to view width)
        val maxAngle = 35f
        val normalizedPosition = (angle / maxAngle).coerceIn(-1f, 1f)
        val centerX = binding.rudderIndicatorContainer.width / 2f
        val maxOffset = binding.rudderIndicatorContainer.width / 2f - binding.rudderIndicator.width / 2f
        binding.rudderIndicator.translationX = normalizedPosition * maxOffset
    }

    private fun normalizeAngle(angle: Float): Float {
        var result = angle
        while (result < -180) result += 360
        while (result >= 180) result -= 360
        return result
    }

    // =========================================================================
    // DIALOGS
    // =========================================================================

    private fun showConnectDialog(device: BluetoothDevice) {
        AlertDialog.Builder(this)
            .setTitle("Device Found")
            .setMessage("Connect to ${device.name ?: device.address}?")
            .setPositiveButton("Connect") { _, _ ->
                bleManager.connect(device)
            }
            .setNegativeButton("Cancel") { _, _ ->
                binding.btnConnect.isEnabled = true
                binding.btnConnect.text = getString(R.string.connect)
            }
            .setCancelable(false)
            .show()
    }

    private fun showCalibrationDialog() {
        val options = arrayOf(
            getString(R.string.cal_center),
            getString(R.string.cal_port),
            getString(R.string.cal_stbd),
            getString(R.string.cal_save),
            getString(R.string.cal_exit)
        )

        AlertDialog.Builder(this)
            .setTitle(getString(R.string.cal_title))
            .setItems(options) { _, which ->
                when (which) {
                    0 -> bleManager.calibrateCenter()
                    1 -> bleManager.calibratePort()
                    2 -> bleManager.calibrateStarboard()
                    3 -> bleManager.saveCalibration()
                    4 -> bleManager.exitCalibration()
                }
            }
            .setNegativeButton("Cancel", null)
            .show()
    }
}
