package com.cascade.testap2

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.*
import kotlin.math.sin

/**
 * BLE Manager for Cascade Autopilot (TestAP2)
 *
 * Handles scanning, connection, and communication with the autopilot device.
 * Supports Demo Mode for testing without hardware.
 *
 * Protocol matches TestAP2 firmware BLE implementation.
 */
@SuppressLint("MissingPermission")
class AutopilotBleManager(private val context: Context) {

    companion object {
        private const val TAG = "AutopilotBLE"

        // Device names to scan for (supports both old and new firmware)
        val DEVICE_NAMES = listOf("CascadeAP", "TestAP2")

        // Service and Characteristic UUIDs (must match firmware)
        val SERVICE_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789abc")
        val CHAR_COMMAND_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789001")
        val CHAR_STATUS_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789002")
        val CHAR_HEADING_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789003")
        val CHAR_RUDDER_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789004")
        val CHAR_PARAMS_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789005")
        val CHAR_MAGCAL_UUID: UUID = UUID.fromString("12345678-1234-1234-1234-123456789006")

        // Client Characteristic Configuration Descriptor
        val CCCD_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        // Command bytes (must match firmware ble_manager.h)
        const val CMD_ENGAGE: Byte = 0x01
        const val CMD_DISENGAGE: Byte = 0x02
        const val CMD_SET_HEADING: Byte = 0x03
        const val CMD_ADJUST_HEADING: Byte = 0x04
        const val CMD_CAL_ENTER: Byte = 0x10
        const val CMD_CAL_EXIT: Byte = 0x11
        const val CMD_CAL_CENTER: Byte = 0x12
        const val CMD_CAL_PORT: Byte = 0x13
        const val CMD_CAL_STBD: Byte = 0x14
        const val CMD_CAL_SAVE: Byte = 0x15
        const val CMD_FAULT_CLEAR: Byte = 0x20
        const val CMD_STATUS_REQ: Byte = 0x30
        const val CMD_PARAM_GET: Byte = 0x50
        const val CMD_PARAM_SET: Byte = 0x51
        const val CMD_PARAM_SAVE: Byte = 0x52
        const val CMD_PARAM_RESET: Byte = 0x53

        // Mag calibration commands
        const val CMD_MAGCAL_GET: Byte = 0x60
        const val CMD_MAGCAL_SET: Byte = 0x61
        const val CMD_MAGCAL_SAVE: Byte = 0x62
        const val CMD_MAGCAL_RESET: Byte = 0x63

        // Scan timeout
        private const val SCAN_TIMEOUT_MS = 10000L

        // Demo mode update rate
        private const val DEMO_UPDATE_MS = 100L
    }

    // Callbacks
    var onConnectionStateChanged: ((Boolean) -> Unit)? = null
    var onStatusReceived: ((AutopilotStatus) -> Unit)? = null
    var onHeadingReceived: ((Float, Float) -> Unit)? = null
    var onRudderReceived: ((Float, Int) -> Unit)? = null
    var onParamReceived: ((ParamData) -> Unit)? = null
    var onMagCalReceived: ((MagCalData) -> Unit)? = null
    var onDeviceFound: ((BluetoothDevice) -> Unit)? = null
    var onScanStopped: (() -> Unit)? = null
    var onError: ((String) -> Unit)? = null

    // Bluetooth components
    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val manager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        manager.adapter
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var commandCharacteristic: BluetoothGattCharacteristic? = null
    private var paramsCharacteristic: BluetoothGattCharacteristic? = null
    private var magcalCharacteristic: BluetoothGattCharacteristic? = null

    private val handler = Handler(Looper.getMainLooper())
    private var isScanning = false
    private var isConnected = false

    // Demo mode state
    private var isDemoMode = false
    private var demoStatus = AutopilotStatus()
    private var demoHeading = 180f
    private var demoTargetHeading = 180f
    private var demoRudderAngle = 0f

    private val demoUpdateRunnable = object : Runnable {
        override fun run() {
            if (isDemoMode) {
                updateDemoState()
                handler.postDelayed(this, DEMO_UPDATE_MS)
            }
        }
    }

    // Scanner callback
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            val name = device.name ?: return

            if (name in DEVICE_NAMES) {
                Log.d(TAG, "Found autopilot device '$name': ${device.address}")
                stopScan()
                onDeviceFound?.invoke(device)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "Scan failed with error: $errorCode")
            isScanning = false
            onError?.invoke("Scan failed: $errorCode")
        }
    }

    // GATT Callback
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.d(TAG, "Connected to GATT server")
                    isConnected = true
                    handler.post { onConnectionStateChanged?.invoke(true) }
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.d(TAG, "Disconnected from GATT server")
                    isConnected = false
                    handler.post { onConnectionStateChanged?.invoke(false) }
                    cleanup()
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d(TAG, "Services discovered")
                setupCharacteristics(gatt)
            } else {
                Log.e(TAG, "Service discovery failed: $status")
                handler.post { onError?.invoke("Service discovery failed") }
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            handleCharacteristicData(characteristic.uuid, value)
        }

        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            characteristic.value?.let { value ->
                handleCharacteristicData(characteristic.uuid, value)
            }
        }

        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                handleCharacteristicData(characteristic.uuid, value)
            }
        }

        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                characteristic.value?.let { value ->
                    handleCharacteristicData(characteristic.uuid, value)
                }
            }
        }
    }

    private fun handleCharacteristicData(uuid: UUID, value: ByteArray) {
        when (uuid) {
            CHAR_STATUS_UUID -> {
                val status = parseStatus(value)
                handler.post { onStatusReceived?.invoke(status) }
            }
            CHAR_HEADING_UUID -> {
                if (value.size >= 4) {
                    val current = ByteBuffer.wrap(value, 0, 2).order(ByteOrder.LITTLE_ENDIAN).short / 10f
                    val target = ByteBuffer.wrap(value, 2, 2).order(ByteOrder.LITTLE_ENDIAN).short / 10f
                    handler.post { onHeadingReceived?.invoke(current, target) }
                }
            }
            CHAR_RUDDER_UUID -> {
                if (value.size >= 3) {
                    val angle = ByteBuffer.wrap(value, 0, 2).order(ByteOrder.LITTLE_ENDIAN).short / 10f
                    val motorStatus = value[2].toInt() and 0xFF
                    handler.post { onRudderReceived?.invoke(angle, motorStatus) }
                }
            }
            CHAR_PARAMS_UUID -> {
                if (value.size >= 13) {
                    val param = parseParam(value)
                    handler.post { onParamReceived?.invoke(param) }
                }
            }
            CHAR_MAGCAL_UUID -> {
                parseMagCalData(value)?.let { magCal ->
                    handler.post { onMagCalReceived?.invoke(magCal) }
                }
            }
        }
    }

    // =========================================================================
    // PUBLIC API
    // =========================================================================

    fun startScan() {
        if (isScanning || isDemoMode) return

        val scanner = bluetoothAdapter?.bluetoothLeScanner
        if (scanner == null) {
            onError?.invoke("Bluetooth not available")
            return
        }

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        Log.d(TAG, "Starting BLE scan...")
        isScanning = true
        scanner.startScan(null, settings, scanCallback)

        handler.postDelayed({
            if (isScanning) {
                stopScan()
                onScanStopped?.invoke()
            }
        }, SCAN_TIMEOUT_MS)
    }

    fun stopScan() {
        if (!isScanning) return
        bluetoothAdapter?.bluetoothLeScanner?.stopScan(scanCallback)
        isScanning = false
        Log.d(TAG, "Scan stopped")
    }

    fun connect(device: BluetoothDevice) {
        Log.d(TAG, "Connecting to ${device.address}...")
        bluetoothGatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
    }

    fun disconnect() {
        if (isDemoMode) {
            exitDemoMode()
        } else {
            bluetoothGatt?.disconnect()
        }
    }

    fun isConnected(): Boolean = isConnected || isDemoMode

    fun isDemoMode(): Boolean = isDemoMode

    // =========================================================================
    // DEMO MODE
    // =========================================================================

    fun enterDemoMode() {
        if (isDemoMode) return

        Log.d(TAG, "Entering Demo Mode")
        isDemoMode = true
        demoStatus = AutopilotStatus(state = AutopilotState.IDLE, calibrationValid = true)
        demoHeading = 180f
        demoTargetHeading = 180f
        demoRudderAngle = 0f

        handler.post {
            onConnectionStateChanged?.invoke(true)
            onStatusReceived?.invoke(demoStatus)
            onHeadingReceived?.invoke(demoHeading, demoTargetHeading)
            onRudderReceived?.invoke(demoRudderAngle, 0)
        }

        handler.postDelayed(demoUpdateRunnable, DEMO_UPDATE_MS)
    }

    private fun exitDemoMode() {
        if (!isDemoMode) return

        Log.d(TAG, "Exiting Demo Mode")
        isDemoMode = false
        handler.removeCallbacks(demoUpdateRunnable)
        handler.post { onConnectionStateChanged?.invoke(false) }
    }

    private fun updateDemoState() {
        // Simulate heading changes when engaged
        if (demoStatus.state == AutopilotState.ENGAGED) {
            // Simulate yaw disturbance
            val disturbance = (sin(System.currentTimeMillis() / 2000.0) * 3).toFloat()
            val error = normalizeAngle(demoTargetHeading - demoHeading)

            // Simulate rudder response
            demoRudderAngle = (error * 0.5f).coerceIn(-30f, 30f)

            // Simulate heading correction
            demoHeading = normalizeAngle360(demoHeading + demoRudderAngle * 0.02f + disturbance * 0.01f)
        }

        onHeadingReceived?.invoke(demoHeading, demoTargetHeading)
        onRudderReceived?.invoke(demoRudderAngle, if (demoRudderAngle != 0f) 0x03 else 0x01)
    }

    // =========================================================================
    // COMMANDS
    // =========================================================================

    fun engage(): Boolean {
        if (isDemoMode) {
            if (demoStatus.state == AutopilotState.IDLE) {
                demoTargetHeading = demoHeading
                demoStatus = demoStatus.copy(state = AutopilotState.ENGAGED)
                handler.post { onStatusReceived?.invoke(demoStatus) }
            }
            return true
        }
        return sendCommand(byteArrayOf(CMD_ENGAGE))
    }

    fun disengage(): Boolean {
        if (isDemoMode) {
            if (demoStatus.state == AutopilotState.ENGAGED) {
                demoStatus = demoStatus.copy(state = AutopilotState.IDLE)
                demoRudderAngle = 0f
                handler.post {
                    onStatusReceived?.invoke(demoStatus)
                    onRudderReceived?.invoke(demoRudderAngle, 0x01)
                }
            }
            return true
        }
        return sendCommand(byteArrayOf(CMD_DISENGAGE))
    }

    fun setHeading(heading: Float): Boolean {
        if (isDemoMode) {
            demoTargetHeading = normalizeAngle360(heading)
            handler.post { onHeadingReceived?.invoke(demoHeading, demoTargetHeading) }
            return true
        }
        // Big-endian format for SET_HEADING
        val headingInt = (heading * 10).toInt()
        val data = byteArrayOf(
            CMD_SET_HEADING,
            ((headingInt shr 8) and 0xFF).toByte(),
            (headingInt and 0xFF).toByte()
        )
        return sendCommand(data)
    }

    fun adjustHeading(offset: Int): Boolean {
        if (isDemoMode) {
            demoTargetHeading = normalizeAngle360(demoTargetHeading + offset)
            handler.post { onHeadingReceived?.invoke(demoHeading, demoTargetHeading) }
            return true
        }
        return sendCommand(byteArrayOf(CMD_ADJUST_HEADING, offset.toByte()))
    }

    fun enterCalibration(): Boolean {
        if (isDemoMode) {
            demoStatus = demoStatus.copy(state = AutopilotState.CALIBRATION)
            handler.post { onStatusReceived?.invoke(demoStatus) }
            return true
        }
        return sendCommand(byteArrayOf(CMD_CAL_ENTER))
    }

    fun exitCalibration(): Boolean {
        if (isDemoMode) {
            demoStatus = demoStatus.copy(state = AutopilotState.IDLE, calibrationValid = true)
            handler.post { onStatusReceived?.invoke(demoStatus) }
            return true
        }
        return sendCommand(byteArrayOf(CMD_CAL_EXIT))
    }

    fun calibrateCenter(): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_CAL_CENTER))
    }

    fun calibratePort(): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_CAL_PORT))
    }

    fun calibrateStarboard(): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_CAL_STBD))
    }

    fun saveCalibration(): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_CAL_SAVE))
    }

    fun clearFault(): Boolean {
        if (isDemoMode) {
            demoStatus = demoStatus.copy(state = AutopilotState.IDLE, fault = FaultCode.NONE)
            handler.post { onStatusReceived?.invoke(demoStatus) }
            return true
        }
        return sendCommand(byteArrayOf(CMD_FAULT_CLEAR))
    }

    fun requestStatus(): Boolean {
        if (isDemoMode) {
            handler.post {
                onStatusReceived?.invoke(demoStatus)
                onHeadingReceived?.invoke(demoHeading, demoTargetHeading)
                onRudderReceived?.invoke(demoRudderAngle, 0)
            }
            return true
        }
        return sendCommand(byteArrayOf(CMD_STATUS_REQ))
    }

    fun getParam(paramId: Int): Boolean {
        if (isDemoMode) {
            // Return default param values in demo mode
            val defaultParams = mapOf(
                0 to ParamData(0, 0.8f, 0.1f, 5.0f, "Kp Heading"),
                1 to ParamData(1, 0.05f, 0.0f, 1.0f, "Ki Heading"),
                2 to ParamData(2, 0.5f, 0.0f, 5.0f, "Kd Heading"),
                3 to ParamData(3, 10.0f, 1.0f, 50.0f, "Kp Servo"),
                4 to ParamData(4, 1.0f, 0.1f, 5.0f, "Deadband Enter"),
                5 to ParamData(5, 1.5f, 0.2f, 6.0f, "Deadband Exit"),
                6 to ParamData(6, 20f, 0f, 50f, "Min Motor Speed"),
                7 to ParamData(7, 100f, 50f, 100f, "Max Motor Speed"),
                8 to ParamData(8, 15.0f, 1.0f, 60.0f, "Slew Rate")
            )
            defaultParams[paramId]?.let { param ->
                handler.post { onParamReceived?.invoke(param) }
            }
            return true
        }
        return sendCommand(byteArrayOf(CMD_PARAM_GET, paramId.toByte()))
    }

    fun setParam(paramId: Int, value: Float): Boolean {
        if (isDemoMode) return true
        val data = ByteArray(6)
        data[0] = CMD_PARAM_SET
        data[1] = paramId.toByte()
        ByteBuffer.wrap(data, 2, 4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value)
        return sendCommand(data)
    }

    fun saveParam(paramId: Int): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_PARAM_SAVE, paramId.toByte()))
    }

    fun saveAllParams(): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_PARAM_SAVE, 0xFF.toByte()))
    }

    fun resetParams(): Boolean {
        if (isDemoMode) return true
        return sendCommand(byteArrayOf(CMD_PARAM_RESET))
    }

    // =========================================================================
    // MAGNETOMETER CALIBRATION
    // =========================================================================

    /**
     * Read magnetometer calibration from device
     * Result will be delivered via onMagCalReceived callback
     */
    fun readMagCal(): Boolean {
        if (isDemoMode) {
            // Return default calibration in demo mode
            handler.post { onMagCalReceived?.invoke(MagCalData()) }
            return true
        }

        val gatt = bluetoothGatt ?: return false
        val char = magcalCharacteristic ?: return false

        return gatt.readCharacteristic(char)
    }

    /**
     * Write magnetometer calibration to device
     * Does NOT persist to NVS - call saveMagCal() to persist
     */
    fun setMagCal(magCal: MagCalData): Boolean {
        if (isDemoMode) return true

        val gatt = bluetoothGatt ?: return false
        val char = magcalCharacteristic ?: return false

        // Build data: cmd(1) + hard_iron(12) + soft_iron(36) = 49 bytes
        val data = ByteArray(49)
        data[0] = CMD_MAGCAL_SET

        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        buffer.position(1)
        buffer.putFloat(magCal.hardIronX)
        buffer.putFloat(magCal.hardIronY)
        buffer.putFloat(magCal.hardIronZ)
        magCal.softIron.forEach { buffer.putFloat(it) }

        char.value = data
        return gatt.writeCharacteristic(char)
    }

    /**
     * Save current magnetometer calibration to device NVS
     */
    fun saveMagCal(): Boolean {
        if (isDemoMode) return true

        val gatt = bluetoothGatt ?: return false
        val char = magcalCharacteristic ?: return false

        char.value = byteArrayOf(CMD_MAGCAL_SAVE)
        return gatt.writeCharacteristic(char)
    }

    /**
     * Reset magnetometer calibration to defaults (zero offsets, identity matrix)
     * Does NOT persist to NVS - call saveMagCal() to persist
     */
    fun resetMagCal(): Boolean {
        if (isDemoMode) {
            handler.post { onMagCalReceived?.invoke(MagCalData()) }
            return true
        }

        val gatt = bluetoothGatt ?: return false
        val char = magcalCharacteristic ?: return false

        char.value = byteArrayOf(CMD_MAGCAL_RESET)
        return gatt.writeCharacteristic(char)
    }

    /**
     * Parse magnetometer calibration data from characteristic read
     */
    private fun parseMagCalData(data: ByteArray): MagCalData? {
        if (data.size < 49) return null

        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        buffer.position(1)  // Skip command byte

        val hardX = buffer.float
        val hardY = buffer.float
        val hardZ = buffer.float

        val soft = FloatArray(9)
        for (i in 0 until 9) {
            soft[i] = buffer.float
        }

        return MagCalData(hardX, hardY, hardZ, soft)
    }

    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================

    private fun setupCharacteristics(gatt: BluetoothGatt) {
        val service = gatt.getService(SERVICE_UUID)
        if (service == null) {
            Log.e(TAG, "Autopilot service not found")
            handler.post { onError?.invoke("Autopilot service not found") }
            return
        }

        commandCharacteristic = service.getCharacteristic(CHAR_COMMAND_UUID)
        paramsCharacteristic = service.getCharacteristic(CHAR_PARAMS_UUID)
        magcalCharacteristic = service.getCharacteristic(CHAR_MAGCAL_UUID)

        // Debug: Log discovered characteristics
        Log.d(TAG, "Discovered characteristics:")
        service.characteristics.forEach { char ->
            Log.d(TAG, "  - ${char.uuid}")
        }
        Log.d(TAG, "magcalCharacteristic found: ${magcalCharacteristic != null}")

        // Enable notifications for status, heading, rudder, params
        listOf(CHAR_STATUS_UUID, CHAR_HEADING_UUID, CHAR_RUDDER_UUID, CHAR_PARAMS_UUID).forEach { uuid ->
            service.getCharacteristic(uuid)?.let { char ->
                gatt.setCharacteristicNotification(char, true)
                char.getDescriptor(CCCD_UUID)?.let { desc ->
                    desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    gatt.writeDescriptor(desc)
                }
            }
        }

        // Request initial status
        handler.postDelayed({ requestStatus() }, 500)

        Log.d(TAG, "Characteristics configured")
    }

    private fun sendCommand(data: ByteArray): Boolean {
        val gatt = bluetoothGatt ?: return false
        val char = commandCharacteristic ?: return false

        char.value = data
        char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        val result = gatt.writeCharacteristic(char)

        Log.d(TAG, "Command sent: ${data.joinToString { String.format("%02X", it) }} - $result")
        return result
    }

    private fun parseStatus(data: ByteArray): AutopilotStatus {
        if (data.size < 4) return AutopilotStatus()

        val state = AutopilotState.fromByte(data[0])
        val fault = FaultCode.fromByte(data[1])
        val flags = data[2].toInt() and 0xFF
        val calValid = (flags and 0x02) != 0

        return AutopilotStatus(
            state = state,
            fault = fault,
            calibrationValid = calValid
        )
    }

    private fun parseParam(data: ByteArray): ParamData {
        val id = data[0].toInt() and 0xFF
        val value = ByteBuffer.wrap(data, 1, 4).order(ByteOrder.LITTLE_ENDIAN).float
        val min = ByteBuffer.wrap(data, 5, 4).order(ByteOrder.LITTLE_ENDIAN).float
        val max = ByteBuffer.wrap(data, 9, 4).order(ByteOrder.LITTLE_ENDIAN).float
        return ParamData(id, value, min, max)
    }

    private fun cleanup() {
        commandCharacteristic = null
        paramsCharacteristic = null
        magcalCharacteristic = null
        bluetoothGatt?.close()
        bluetoothGatt = null
    }

    fun close() {
        stopScan()
        if (isDemoMode) exitDemoMode()
        disconnect()
        cleanup()
    }

    private fun normalizeAngle(angle: Float): Float {
        var result = angle
        while (result < -180) result += 360
        while (result >= 180) result -= 360
        return result
    }

    private fun normalizeAngle360(angle: Float): Float {
        var result = angle
        while (result < 0) result += 360
        while (result >= 360) result -= 360
        return result
    }
}

/**
 * Autopilot system states
 */
enum class AutopilotState(val value: Byte) {
    BOOT(0),
    IDLE(1),
    ENGAGED(2),
    CALIBRATION(3),
    FAULTED(0xFF.toByte()),
    UNKNOWN(-1);

    companion object {
        fun fromByte(b: Byte): AutopilotState {
            return entries.find { it.value == b } ?: UNKNOWN
        }
    }
}

/**
 * Fault codes (matches firmware can_protocol.h)
 */
enum class FaultCode(val value: Byte) {
    NONE(0x00),
    CAN_TX_FAIL(0x01),
    CAN_RX_TIMEOUT(0x02),
    CAN_BUS_OFF(0x03),
    SENSOR_FAULT(0x10),
    SENSOR_RANGE(0x11),
    SENSOR_INIT(0x12),
    MOTOR_STALL(0x20),
    MOTOR_OVERCURRENT(0x21),
    MOTOR_TIMEOUT(0x22),
    CAL_INVALID(0x30),
    CAL_RANGE(0x31),
    HEARTBEAT_LOST(0x40),
    STATE_MISMATCH(0x41),
    LOW_VOLTAGE(0x50),
    OVER_VOLTAGE(0x51),
    WATCHDOG(0xFE.toByte()),
    UNKNOWN(0xFF.toByte());

    companion object {
        fun fromByte(b: Byte): FaultCode {
            return entries.find { it.value == b } ?: UNKNOWN
        }
    }
}

/**
 * Autopilot status data
 */
data class AutopilotStatus(
    val state: AutopilotState = AutopilotState.UNKNOWN,
    val fault: FaultCode = FaultCode.NONE,
    val calibrationValid: Boolean = false
)

/**
 * Parameter data
 */
data class ParamData(
    val id: Int,
    val value: Float,
    val min: Float,
    val max: Float,
    val name: String = getParamName(id)
) {
    companion object {
        fun getParamName(id: Int): String = when (id) {
            0 -> "Kp Heading"
            1 -> "Ki Heading"
            2 -> "Kd Heading"
            3 -> "Kp Servo"
            4 -> "Deadband Enter"
            5 -> "Deadband Exit"
            6 -> "Min Motor Speed"
            7 -> "Max Motor Speed"
            8 -> "Slew Rate"
            else -> "Param $id"
        }
    }
}

/**
 * Magnetometer calibration data
 * Hard-iron: 3 floats (X, Y, Z offsets in uT)
 * Soft-iron: 9 floats (3x3 correction matrix in row-major order)
 */
data class MagCalData(
    val hardIronX: Float = 0f,
    val hardIronY: Float = 0f,
    val hardIronZ: Float = 0f,
    val softIron: FloatArray = floatArrayOf(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f)
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as MagCalData
        if (hardIronX != other.hardIronX) return false
        if (hardIronY != other.hardIronY) return false
        if (hardIronZ != other.hardIronZ) return false
        if (!softIron.contentEquals(other.softIron)) return false
        return true
    }

    override fun hashCode(): Int {
        var result = hardIronX.hashCode()
        result = 31 * result + hardIronY.hashCode()
        result = 31 * result + hardIronZ.hashCode()
        result = 31 * result + softIron.contentHashCode()
        return result
    }

    fun isIdentity(): Boolean {
        return hardIronX == 0f && hardIronY == 0f && hardIronZ == 0f &&
               softIron.contentEquals(floatArrayOf(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f))
    }
}
