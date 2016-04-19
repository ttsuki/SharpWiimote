/**
 * Nintendo Wiimote (RVL-CNT-01-TR) I/O library.
 * for Windows 8+/Microsoft Bluetooth stack.
 * 2016.03.11 / http://tu3.jp/
 *
 * Copyright (c) 2016 tu-sa
 *
 * This software is released under the MIT License.
 * http://opensource.org/licenses/mit-license.php
 */

// TestCode
//#define WIIMOTE_DEVICE_TEST

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.Permissions;
using System.Threading;
using Microsoft.Win32.SafeHandles;

/// <summary>
/// Wiiリモコン
/// </summary>
namespace Tsukikage.Wiimote
{
#if WIIMOTE_DEVICE_TEST
    public static class WiimoteDeviceTest
    {
        public static void RunTest()
        {
            string[] wiimotes = WiimoteDevice.FindAllWiimoteDevicePath();

            if (wiimotes.Length > 0)
            {
                Console.WriteLine(wiimotes[0]);
                using (WiimoteDevice remocon = new WiimoteDevice(wiimotes[0]))
                {
                    remocon.StatusChanged += (o, stt) =>
                    {
                        Console.WriteLine(
                            stt.TimeStamp.ToString("HH:mm:ss.fff")
                            + ">"
                            + " Btn:" + stt.Buttons
                            + " Accel: " + stt.Accelerometer
                            + " Gyro: " + stt.GyroSensor
                            + " Battery: " + (stt.Battery * 100).ToString("0.0")
                            + " IR: " + (stt.IRSensor.Position0.HasValue ? stt.IRSensor.Position0.Value.ToString() : "-")
                            );
                    };
                    remocon.OpenDevice();
                    remocon.ActivateMotionPlus();
                    Console.ReadLine();
                }
            }
        }
    }
#endif

    /// <summary>
    /// Wiimote Status
    /// </summary>
    public struct WiimoteStatus
    {
        /// <summary>
        /// Vector
        /// </summary>
        public struct Vector3F
        {
            /// <summary>
            /// X,Y,Z or yaw,pitch,roll.
            /// </summary>
            public float X, Y, Z;
            public Vector3F(float x, float y, float z) { X = x; Y = y; Z = z; }
            public override string ToString()
            {
                return "{"
                    + X.ToString("+0.000;-0.000").PadLeft(7) + ","
                    + Y.ToString("+0.000;-0.000").PadLeft(7) + ","
                    + Z.ToString("+0.000;-0.000").PadLeft(7) + "}";
            }
        }

        /// <summary>
        /// Buttons
        /// </summary>
        public struct ButtonStatus
        {
            /// <summary>
            /// Button is pressed or not.
            /// </summary>
            public bool A, B, Plus, Home, Minus, One, Two, Up, Down, Left, Right;

            public override string ToString()
            {
                return ""
                    + (A ? "A" : "_")
                    + (B ? "B" : "_")
                    + (Plus ? "+" : "_")
                    + (Home ? "H" : "_")
                    + (Minus ? "-" : "_")
                    + (One ? "1" : "_")
                    + (Two ? "2" : "_")
                    + (Up ? "^" : "_")
                    + (Down ? "v" : "_")
                    + (Left ? "<" : "_")
                    + (Right ? ">" : "_");
            }
        }

        /// <summary>
        /// IR sensor.
        /// </summary>
        public struct IRSensorStatus
        {
            /// <summary>
            /// IR detected point on (X-Y) in [0.0-1.0] or Null if not detected.
            /// </summary>
            public Vector3F? Position0;

            /// <summary>
            /// IR detected point on (X-Y) in [0.0-1.0] or Null if not detected.
            /// </summary>
            public Vector3F? Position1;

            /// <summary>
            /// IR detected point on (X-Y) in [0.0-1.0] or Null if not detected.
            /// </summary>
            public Vector3F? Position2;

            /// <summary>
            /// IR detected point on (X-Y) in [0.0-1.0] or Null if not detected.
            /// </summary>
            public Vector3F? Position3;

            public override string ToString()
            {
                return "{"
                    + (Position0.HasValue ? Position0.Value + "," : "-,")
                    + (Position1.HasValue ? Position1.Value + "," : "-,")
                    + (Position2.HasValue ? Position2.Value + "," : "-,")
                    + (Position3.HasValue ? Position3.Value + "," : "-,") + "}";
            }
        }

        /// <summary>
        /// Timestamp.
        /// </summary>
        public DateTime TimeStamp;

        /// <summary>
        /// Buttons data.
        /// </summary>
        public ButtonStatus Buttons;

        /// <summary>
        /// Accelerometer sensors data (X,Y,Z). 1.00f = 1.00G.
        /// </summary>
        public Vector3F Accelerometer;

        /// <summary>
        /// MotionPlus data (yaw,pitch,roll). deg/s
        /// </summary>
        public Vector3F Gyroscope;

        /// <summary>
        /// IR sensor data. 
        /// </summary>
        public IRSensorStatus IRSensor;

        /// <summary>
        /// Battery data.
        /// </summary>
        public float Battery;
    }

    /// <summary>
    /// Handles device status changed event.
    /// </summary>
    /// <param name="sender">Sender.</param>
    /// <param name="status">New device status.</param>
    public delegate void WiimoteStatusChangedDelegate(object sender, WiimoteStatus status);

    /// <summary>
    /// Wiimote
    /// </summary>
    public class WiimoteDevice : IDisposable
    {
        enum ExtensionType : long
        {
            None = 0x000000000000,
            Error = 0xFFFFFFFFFFFF,
            MotionPlus = 0x0000A4200405,
            MotionPlusEx = 0x0100A4200405,
        };

        enum ReportType : byte
        {
            B2 = 0x30,
            B2A3 = 0x31,
            B2A3I12 = 0x33,
            B2E19 = 0x34,
            B2A3E16 = 0x35,
            B2I10E9 = 0x36,
            B2A3I10E6 = 0x37,
            E21 = 0x3d,
            B2A118_1 = 0x3e,
            B2A118_2 = 0x3f,
        };

        /// <summary>
        /// Device Path
        /// </summary>
        public string DevicePath { get; private set; }

        WiimoteIO device = null;
        WiimoteStatus lastStatus = new WiimoteStatus();
        bool rumbleStatus = false;
        int ledStatus = 0;

        byte[] writeBuffer = new byte[22];

        ManualResetEvent readMemoryReady = null;
        byte[] readBuffer = new byte[32];
        int readOffset = 0;
        int readSize = 0;
        Action<byte[]> readOnData = null;

        ExtensionType connectedExtension = ExtensionType.None;

        struct AccelSensorCalibrationValue
        {
            public bool Initialized;
            public WiimoteStatus.Vector3F G;
            public WiimoteStatus.Vector3F Origin;
        }

        struct MotionPlusCalibrationValue
        {
            public WiimoteStatus.Vector3F totalValue;
            public int sampleCount;
        }

        AccelSensorCalibrationValue accelSensorCalibrationData = new AccelSensorCalibrationValue();
        MotionPlusCalibrationValue motionPlusCalibrationValues = new MotionPlusCalibrationValue();

        /// <summary>
        /// Find all connected wiimote device path
        /// </summary>
        /// <returns></returns>
        public static string[] FindAllWiimoteDevicePath()
        {
            return WiimoteFinder.FindAllWiimoteDevicePath();
        }

        /// <summary>
        /// On device status changed.
        /// </summary>
        public event WiimoteStatusChangedDelegate StatusChanged;

        /// <summary>
        /// Device Clock
        /// </summary>
        DateTime Clock
        {
            get
            {
                ThrowIfNotOpen();
                return device.Clock;
            }
        }

        /// <summary>
        /// Construct
        /// </summary>
        /// <param name="devicePath"></param>
        public WiimoteDevice(string devicePath)
        {
            this.DevicePath = devicePath;
        }

        /// <summary>
        /// Open Wiimote device and initialize it.
        /// </summary>
        public void OpenDevice()
        {
            Debug.Assert(device == null);

            CloseDevice();
            readMemoryReady = new ManualResetEvent(true);
            device = new WiimoteIO(DevicePath, ParseDeviceReport);
            SetLED(0);

            // Calibrate accelSensor
            ReadMemory(0x0016, 8, data =>
            {
                AccelSensorCalibrationValue cv = new AccelSensorCalibrationValue()
                {
                    Origin = ParseAccelCalibration(data, 0),
                    G = ParseAccelCalibration(data, 4),
                };

                cv.G.X -= cv.Origin.X;
                cv.G.Y -= cv.Origin.Y;
                cv.G.Z -= cv.Origin.Z;
                cv.Initialized = true;
                accelSensorCalibrationData = cv;
            }
            );
            WaitReadMemory();

            SetReportType(ReportType.B2A3I12, true);
            RequestStatus();
        }

        /// <summary>
        /// Close Wiimote device.
        /// </summary>
        public void CloseDevice()
        {
            if (device != null)
            {
                SetReportType(ReportType.B2, true);
                device.Flush();
                device.Dispose();
                device = null;
            }

            if (readMemoryReady != null)
            {
                readMemoryReady.Close();
                readMemoryReady = null;
            }
        }

        /// <summary>
        /// Dispose Object
        /// </summary>
        public void Dispose()
        {
            CloseDevice();
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Set LED Status
        /// </summary>
        /// <param name="leds">1,2,4,8</param>
        public void SetLED(int leds)
        {
            ThrowIfNotOpen();
            ledStatus = leds & 0xF;

            writeBuffer[0] = (byte)0x11;
            writeBuffer[1] = (byte)((ledStatus << 4) | (rumbleStatus ? 1 : 0));
            device.Write(writeBuffer, 2);
        }

        /// <summary>
        /// Set Vibration Status.
        /// </summary>
        /// <param name="stat"></param>
        public void SetRumble(bool stat)
        {
            ThrowIfNotOpen();
            rumbleStatus = stat;
            SetLED(ledStatus);
        }

        /// <summary>
        /// Initialize Motion Plus
        /// </summary>
        public void ActivateMotionPlus()
        {
            ThrowIfNotOpen();
            WriteMemory(0x04a600fe, 0x04);
            CalibrateMotionPlus();
        }

        /// <summary>
        /// Re-calibrate Motion Plus.
        /// </summary>
        public void CalibrateMotionPlus()
        {
            motionPlusCalibrationValues = new MotionPlusCalibrationValue();
        }


        void SetReportType(ReportType type, bool continuous)
        {
            switch (type)
            {
                case ReportType.B2:
                case ReportType.B2A3I12:
                case ReportType.B2A3I10E6:
                    break;
                default:
                    // 他もサポートしてないわけではないが...
                    throw new NotSupportedException();
            }

            // Set IR Camera mode
            byte irMode = 0;
            switch (type)
            {
                case ReportType.B2I10E9: irMode = 1; break; // basic
                case ReportType.B2A3I10E6: irMode = 1; break; // basic
                case ReportType.B2A3I12: irMode = 3; break; // extended
            }

            writeBuffer[0] = 0x13; // IR
            writeBuffer[1] = (byte)((irMode != 0 ? 0x04 : 0x00) | (rumbleStatus ? 1 : 0));
            device.Write(writeBuffer, 2);

            writeBuffer[0] = 0x1a; // IR2
            writeBuffer[1] = writeBuffer[1];
            device.Write(writeBuffer, 2);

            if (irMode != 0)
            {
                WriteMemory(0x04b00030, 0x08);
                WriteMemory(0x04b00000, 9, new byte[] { 0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xAA, 0x00, 0x64 }); // Wii Level 3
                WriteMemory(0x04b0001a, 2, new byte[] { 0x63, 0x03 }); // Wii Level 3
                WriteMemory(0x04b00033, irMode);
                WriteMemory(0x04b00030, 0x08);
            }

            // Set Report type
            writeBuffer[0] = (byte)0x12;
            writeBuffer[1] = (byte)((continuous ? 4 : 0) | (rumbleStatus ? 1 : 0));
            writeBuffer[2] = (byte)type;
            device.Write(writeBuffer, 3);
        }

        void ThrowIfNotOpen()
        {
            if (device == null) { throw new InvalidOperationException("Device is not open."); }
        }

        void UpdateConnectedExtension(byte[] statusBytes)
        {
            bool connected = (statusBytes[3] & 0x02) != 0;
            ReadMemory(0x04A400FE, 1, data =>
            {
                const int motionplus = 0x04;
                bool connectedStateChanged = connectedExtension == ExtensionType.None ^ connected;

                if (connectedStateChanged || data[0] == motionplus)
                {
                    InitializeExtension(data[0] != motionplus);
                }
            });
        }

        void InitializeExtension(bool needToWriteMemory)
        {
            if (needToWriteMemory)
            {
                WriteMemory(0x04A400F0, 0x55);
                WriteMemory(0x04A400FB, 0x00);
            }

            // 接続されているExtensionの読み取り
            ReadMemory(0x04A400FA, 6, data =>
            {
                long type = ((long)data[0] << 40) | ((long)data[1] << 32) | ((long)data[2]) << 24 | ((long)data[3]) << 16 | ((long)data[4]) << 8 | data[5];
                switch ((ExtensionType)type)
                {
                    case ExtensionType.None:
                    case ExtensionType.Error:
                        connectedExtension = ExtensionType.None;
                        SetReportType(ReportType.B2A3I12, true);
                        break;
                    case ExtensionType.MotionPlus:
                    case ExtensionType.MotionPlusEx:
                        connectedExtension = (ExtensionType)type;
                        SetReportType(ReportType.B2A3I10E6, true);
                        break;
                    default: // unkown
                        connectedExtension = (ExtensionType)type;
                        SetReportType(ReportType.B2A3I12, true);
                        Debug.WriteLine("Unknown extension controller connected: " + connectedExtension.ToString("x"));
                        break;
                }
            });
        }

        void RequestStatus()
        {
            writeBuffer[0] = (byte)0x15;
            writeBuffer[1] = (byte)(rumbleStatus ? 1 : 0);
            device.Write(writeBuffer, 2);
        }

        void WriteMemory(int addressFrom, byte data)
        {
            Debug.Assert((addressFrom >> 24 & 0xFF) == 0x04);
            Array.Clear(writeBuffer, 0, writeBuffer.Length);
            writeBuffer[0] = (byte)0x16;
            writeBuffer[1] = (byte)(0x04 | (rumbleStatus ? 1 : 0));
            writeBuffer[2] = (byte)(addressFrom >> 16 & 0xFF);
            writeBuffer[3] = (byte)(addressFrom >> 8 & 0xFF);
            writeBuffer[4] = (byte)(addressFrom & 0xFF);
            writeBuffer[5] = 1;
            writeBuffer[6] = data;
            device.Write(writeBuffer, 22);
        }

        void WriteMemory(int addressFrom, int length, byte[] data)
        {
            Debug.Assert((addressFrom >> 24 & 0xFF) == 0x04);
            Array.Clear(writeBuffer, 0, writeBuffer.Length);
            writeBuffer[0] = (byte)0x16;
            writeBuffer[1] = (byte)(0x04 | (rumbleStatus ? 1 : 0));
            writeBuffer[2] = (byte)(addressFrom >> 16 & 0xFF);
            writeBuffer[3] = (byte)(addressFrom >> 8 & 0xFF);
            writeBuffer[4] = (byte)(addressFrom & 0xFF);
            writeBuffer[5] = (byte)(length & 0xFF);
            Array.Copy(data, 0, writeBuffer, 6, length);
            device.Write(writeBuffer, 22);
        }

        void ReadMemory(int addressFrom, int length, Action<byte[]> onData)
        {
            WaitReadMemory();
            readOffset = addressFrom & 0xFFFF;
            readSize = length;
            readOnData = onData;
            Array.Clear(readBuffer, 0, readBuffer.Length);
            writeBuffer[0] = (byte)0x17;
            writeBuffer[1] = (byte)((addressFrom >> 24 & 0xFF) | (rumbleStatus ? 1 : 0));
            writeBuffer[2] = (byte)(addressFrom >> 16 & 0xFF);
            writeBuffer[3] = (byte)(addressFrom >> 8 & 0xFF);
            writeBuffer[4] = (byte)(addressFrom & 0xff);
            writeBuffer[5] = (byte)(length >> 8 & 0xff);
            writeBuffer[6] = (byte)(length & 0xFF);
            device.Write(writeBuffer, 7);
        }

        void WaitReadMemory()
        {
            if (!readMemoryReady.WaitOne(1000))
            {
                throw new IOException("Device didn't respond.");
            }
        }

        void ParseDeviceReport(object sender, byte[] data, int length, DateTime timeStamp)
        {
            bool callStatusChange = false;
            WiimoteStatus status = lastStatus;
            status.TimeStamp = timeStamp;

            switch (data[0])
            {
                case 0x20: // status
                    status.Battery = ParseBattery(data);
                    ledStatus = ParseLED(data);
                    UpdateConnectedExtension(data);
                    break;
                case 0x21: // read memory
                    status.Buttons = ParseButtons(data);
                    ParseReadMemory(data, length);
                    break;
                case 0x22: // ack
                    break;

                case (byte)ReportType.B2:
                    status.Buttons = ParseButtons(data);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.B2A3:
                    status.Buttons = ParseButtons(data);
                    status.Accelerometer = ParseAccel(data);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.B2A3I12:
                    status.Buttons = ParseButtons(data);
                    status.Accelerometer = ParseAccel(data);
                    status.IRSensor = ParseIR(data);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.B2E19:
                    status.Buttons = ParseButtons(data);
                    status.Gyroscope = ParseMotionPlus(data, 3);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.B2A3E16:
                    status.Buttons = ParseButtons(data);
                    status.Accelerometer = ParseAccel(data);
                    status.Gyroscope = ParseMotionPlus(data, 6);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.B2I10E9:
                    status.Buttons = ParseButtons(data);
                    status.IRSensor = ParseIR(data);
                    status.Gyroscope = ParseMotionPlus(data, 13);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.B2A3I10E6:
                    status.Buttons = ParseButtons(data);
                    status.Accelerometer = ParseAccel(data);
                    status.IRSensor = ParseIR(data);
                    status.Gyroscope = ParseMotionPlus(data, 16);
                    callStatusChange = true;
                    break;
                case (byte)ReportType.E21:
                    status.Gyroscope = ParseMotionPlus(data, 1);
                    callStatusChange = true;
                    break;
                default:
                    Debug.WriteLine("Unknown report type: " + data[0].ToString("x"));
                    break;
            }

            if (callStatusChange && StatusChanged != null) { StatusChanged(this, status); }
            lastStatus = status;
        }

        int ParseLED(byte[] statusData)
        {
            return statusData[3] >> 4 & 0xF;
        }

        float ParseBattery(byte[] statusData)
        {
            return statusData[6] / 255.0f;
        }

        WiimoteStatus.ButtonStatus ParseButtons(byte[] data)
        {
            WiimoteStatus.ButtonStatus result = new WiimoteStatus.ButtonStatus();
            result.Plus = (data[1] & 0x10) != 0;
            result.Up = (data[1] & 0x08) != 0;
            result.Down = (data[1] & 0x04) != 0;
            result.Right = (data[1] & 0x02) != 0;
            result.Left = (data[1] & 0x01) != 0;

            result.Home = (data[2] & 0x80) != 0;
            result.Minus = (data[2] & 0x10) != 0;
            result.A = (data[2] & 0x08) != 0;
            result.B = (data[2] & 0x04) != 0;
            result.One = (data[2] & 0x02) != 0;
            result.Two = (data[2] & 0x01) != 0;
            return result;
        }

        WiimoteStatus.IRSensorStatus ParseIR(byte[] data)
        {
            WiimoteStatus.IRSensorStatus result = new WiimoteStatus.IRSensorStatus();
            switch ((ReportType)data[0])
            {
                case ReportType.B2I10E9:
                    ParseIRBasic(data, 3 + 0, out result.Position0, out result.Position1);
                    ParseIRBasic(data, 3 + 5, out result.Position2, out result.Position3);
                    break;
                case ReportType.B2A3I10E6:
                    ParseIRBasic(data, 6 + 0, out result.Position0, out result.Position1);
                    ParseIRBasic(data, 6 + 5, out result.Position2, out result.Position3);
                    break;
                case ReportType.B2A3I12:
                    ParseIRExtended(data, 6 + 0, out result.Position0);
                    ParseIRExtended(data, 6 + 3, out result.Position1);
                    ParseIRExtended(data, 6 + 6, out result.Position2);
                    ParseIRExtended(data, 6 + 9, out result.Position3);
                    break;
            }
            return result;
        }

        void ParseIRExtended(byte[] data, int index, out WiimoteStatus.Vector3F? ret)
        {
            int X = data[index + 0] | data[index + 2] << 4 & 0x0300;
            int Y = data[index + 1] | data[index + 2] << 2 & 0x0300;
            int S = data[index + 2] & 0x0f;
            ret = X != 0x03FF || Y != 0x03FF ? (WiimoteStatus.Vector3F?)new WiimoteStatus.Vector3F((float)X / 0x03FF, (float)Y / 0x03FF, (float)S / 0x0F) : null;
        }

        void ParseIRBasic(byte[] data, int index, out WiimoteStatus.Vector3F? ret, out WiimoteStatus.Vector3F? ret2)
        {
            int X1 = data[index + 0] | data[index + 2] << 4 & 0x0300;
            int Y1 = data[index + 1] | data[index + 2] << 2 & 0x0300;
            int X2 = data[index + 3] | data[index + 2] << 8 & 0x0300;
            int Y2 = data[index + 4] | data[index + 2] << 6 & 0x0300;
            ret = X1 != 0x03FF || Y1 != 0x03FF ? (WiimoteStatus.Vector3F?)new WiimoteStatus.Vector3F((float)X1 / 0x03FF, (float)Y1 / 0x03FF, 0) : null;
            ret2 = X2 != 0x03FF || Y2 != 0x03FF ? (WiimoteStatus.Vector3F?)new WiimoteStatus.Vector3F((float)X2 / 0x03FF, (float)Y2 / 0x03FF, 0) : null;
        }

        WiimoteStatus.Vector3F ParseAccelCalibration(byte[] data, int index)
        {
            return new WiimoteStatus.Vector3F(
                (data[index + 0] << 2 | data[index + 3] >> 4 & 3),
                (data[index + 1] << 2 | data[index + 3] >> 2 & 3),
                (data[index + 2] << 2 | data[index + 3] >> 0 & 3));
        }

        WiimoteStatus.Vector3F ParseAccel(byte[] data)
        {
            AccelSensorCalibrationValue cv = accelSensorCalibrationData;
            if (!cv.Initialized) { return new WiimoteStatus.Vector3F(); }
            return new WiimoteStatus.Vector3F(
                ((data[3] << 2 | data[1] >> 5 & 3) - cv.Origin.X) / cv.G.X,
                ((data[4] << 2 | data[2] >> 4 & 2) - cv.Origin.Y) / cv.G.Y,
                ((data[5] << 2 | data[2] >> 5 & 2) - cv.Origin.Z) / cv.G.Z
                );
        }

        WiimoteStatus.Vector3F ParseMotionPlus(byte[] data, int extensionDataOffset)
        {
            int offset = extensionDataOffset;
            WiimoteStatus.Vector3F ret = new WiimoteStatus.Vector3F();
            if (connectedExtension == ExtensionType.MotionPlus || connectedExtension == ExtensionType.MotionPlusEx)
            {
                bool yawFast = ((data[offset + 3] & 0x02) >> 1) == 0;
                bool pitchFast = ((data[offset + 3] & 0x01) >> 0) == 0;
                bool rollFast = ((data[offset + 4] & 0x02) >> 1) == 0;

                int yawRaw = (data[offset + 0] | (data[offset + 3] & 0xFC) << 6);
                int pitchRaw = (data[offset + 2] | (data[offset + 5] & 0xFC) << 6);
                int rollRaw = (data[offset + 1] | (data[offset + 4] & 0xFC) << 6);

                const int zero = 8192;
                const float degs = 595.0f / 8192.0f;
                const float fastRate = 2000.0f / 440.0f;
                const int calibrationSamples = 100;
                float mpexRate = connectedExtension == ExtensionType.MotionPlusEx ? 90.0f / 128.0f : 1.0f;
                float yaw = (yawRaw - zero) * degs * mpexRate * (yawFast ? fastRate : 1.0f);
                float pitch = (pitchRaw - zero) * degs * mpexRate * (pitchFast ? fastRate : 1.0f);
                float roll = (rollRaw - zero) * degs * mpexRate * (rollFast ? fastRate : 1.0f);

                if (motionPlusCalibrationValues.sampleCount < calibrationSamples)
                {
                    motionPlusCalibrationValues.totalValue.X += yaw;
                    motionPlusCalibrationValues.totalValue.Y += pitch;
                    motionPlusCalibrationValues.totalValue.Z += roll;
                    motionPlusCalibrationValues.sampleCount++;
                }
                else
                {
                    yaw -= motionPlusCalibrationValues.totalValue.X / motionPlusCalibrationValues.sampleCount;
                    pitch -= motionPlusCalibrationValues.totalValue.Y / motionPlusCalibrationValues.sampleCount;
                    roll -= motionPlusCalibrationValues.totalValue.Z / motionPlusCalibrationValues.sampleCount;
                    ret = new WiimoteStatus.Vector3F(yaw, pitch, roll);
                }
            }
            return ret;
        }

        void ParseReadMemory(byte[] data, int length)
        {
            if ((data[3] & 0x0F) != 0)
            {
                readMemoryReady.Set();
                throw new IOException("Device reported an error.");
            }

            int size = (data[3] >> 4) + 1;
            int offset = (data[4] << 8 | data[5]);
            Array.Copy(data, 6, readBuffer, offset - readOffset, size);

            // 全部きた？
            if (readOffset + readSize == offset + size)
            {
                readOffset = 0;
                readSize = 0;
                if (readOnData != null)
                {
                    readOnData(readBuffer);
                }
                readMemoryReady.Set();
            }
        }

    }

    class WiimoteFinder
    {
        private const int VID = 0x057e; // Nintendo
        private const int PID = 0x0306; // Wiimote.
        private const int PID2 = 0x0330; // Newer Wiimote.

        public static string[] FindAllWiimoteDevicePath()
        {
            // Wiiリモコンをさがす
            Guid guid;
            NativeMethods.HidD_GetHidGuid(out guid);
            IntPtr hDevInfo = IntPtr.Zero;
            List<string> foundWiimotes = new List<string>();

            try
            {
                hDevInfo = NativeMethods.SetupDiGetClassDevs(ref guid, IntPtr.Zero, IntPtr.Zero,
                     NativeMethods.SetupDiGetClassFlags.DeviceInterface | NativeMethods.SetupDiGetClassFlags.Present);

                NativeMethods.SP_DEVICE_INTERFACE_DATA diData =
                    new NativeMethods.SP_DEVICE_INTERFACE_DATA()
                    {
                        cbSize = Marshal.SizeOf(typeof(NativeMethods.SP_DEVICE_INTERFACE_DATA))
                    };

                // HIDデバイスを列挙しながら、Wiiリモコンだったらそれをメモってく。
                for (int index = 0; NativeMethods.SetupDiEnumDeviceInterfaces(hDevInfo, IntPtr.Zero, ref guid, index, ref diData); index++)
                {
                    UInt32 size;
                    NativeMethods.SP_DEVICE_INTERFACE_DETAIL_DATA diDetail = new NativeMethods.SP_DEVICE_INTERFACE_DETAIL_DATA() { cbSize = (uint)(IntPtr.Size == 8 ? 8 : 5) };
                    if (!NativeMethods.SetupDiGetDeviceInterfaceDetail(hDevInfo, ref diData, ref diDetail, (uint)Marshal.SizeOf(diDetail), out size, IntPtr.Zero))
                    {
                        // Marshal.ThrowExceptionForHR(Marshal.GetHRForLastWin32Error());
                        Debug.WriteLine("SetupDiGetDeviceInterfaceDetail failed. i = " + index + " error = " + Marshal.GetLastWin32Error());
                        continue;
                    }

                    Debug.WriteLine(string.Format("HID Device: {0}: {1}", index, diDetail.DevicePath));

                    using (SafeHandle handle = NativeMethods.CreateFile(diDetail.DevicePath,
                        NativeMethods.FileAccess.GenericRead | NativeMethods.FileAccess.GenericWrite,
                        FileShare.ReadWrite, IntPtr.Zero, FileMode.Open,
                        NativeMethods.FileAttributes.Overlapped, IntPtr.Zero))
                    {
                        NativeMethods.HIDD_ATTRIBUTES attrib = new NativeMethods.HIDD_ATTRIBUTES() { Size = Marshal.SizeOf(typeof(NativeMethods.HIDD_ATTRIBUTES)) };

                        if (NativeMethods.HidD_GetAttributes(handle.DangerousGetHandle(), ref attrib))
                        {
                            if (attrib.VendorID == VID && (attrib.ProductID == PID || attrib.ProductID == PID2))
                            {
                                foundWiimotes.Add(diDetail.DevicePath);
                            }
                        }
                    }
                }
            }
            finally
            {
                if (hDevInfo != IntPtr.Zero)
                {
                    NativeMethods.SetupDiDestroyDeviceInfoList(hDevInfo);
                }
            }
            return foundWiimotes.ToArray();
        }
    }

    unsafe class WiimoteIO : IDisposable
    {
        public delegate void ReadEventDelegate(object sender, byte[] data, int length, DateTime timeStamp);
        public event ReadEventDelegate DeviceReport;
        public DateTime InitializedAt { get; private set; }
        public DateTime Clock { get { return InitializedAt + stopwatch.Elapsed; } }

        public WiimoteIO(string devicePath, ReadEventDelegate deviceReport)
        {
            deviceHandle = NativeMethods.CreateFile(devicePath,
                        NativeMethods.FileAccess.GenericRead | NativeMethods.FileAccess.GenericWrite,
                        FileShare.ReadWrite, IntPtr.Zero, FileMode.Open,
                        NativeMethods.FileAttributes.Normal | NativeMethods.FileAttributes.Overlapped, IntPtr.Zero);

            if (deviceHandle.IsInvalid)
            {
                Marshal.ThrowExceptionForHR(Marshal.GetHRForLastWin32Error());
            }

            stopwatch.Start();
            InitializedAt = DateTime.Now;

            if (deviceReport != null)
            {
                DeviceReport += deviceReport;
            }

            readerThread = new Thread(ReaderThreadProc);
            readerThread.Name = "Wiimote reader thread (" + devicePath + ")";
            readerThread.Priority = ThreadPriority.Highest;
            readerThread.Start();

            dispatcherThread = new Thread(ReadEventDispatchThreadProc);
            dispatcherThread.Name = "Wiimote dispatcher thread (" + devicePath + ")";
            dispatcherThread.Priority = ThreadPriority.AboveNormal;
            dispatcherThread.Start();

            writerThread = new Thread(WriterThreadProc);
            writerThread.Name = "Wiimote writer thread (" + devicePath + ")";
            writerThread.Priority = ThreadPriority.Highest;
            writerThread.Start();
        }

        public void Dispose()
        {
            if (disposing == null) { return; }

            Flush();
            disposing.Set();

            if (writerThread != null)
            {
                writerThread.Join(10000);
                writerThread = null;
            }

            if (dispatcherThread != null)
            {
                dispatcherThread.Join(10000);
                dispatcherThread = null;
            }

            if (readerThread != null)
            {
                readerThread.Join(10000);
                readerThread = null;
            }

            if (writeCount != null)
            {
                writeCount.Close();
                writeCount = null;
            }

            if (readCount != null)
            {
                readCount.Close();
                readCount = null;
            }

            if (deviceHandle != null)
            {
                deviceHandle.Close();
                deviceHandle = null;
            }

            disposing.Close();
            disposing = null;
        }

        public int Write(byte[] data, int length)
        {
            ThrowOnDisposed();
            if (length > REPORT_SIZE) { return 0; }

            IOEvent e = new IOEvent();
            e.Length = length;
            e.Timestamp = Clock;
            for (int i = 0; i < e.Length; i++)
                e.Data.Data[i] = data[i];

            lock (sending)
            {
                sending.Enqueue(e);
                writeCount.Release();
            }
            return length;
        }

        public void Flush()
        {
            ThrowOnDisposed();
            while (writeCount.WaitOne(0))
            {
                writeCount.Release();
                Thread.Sleep(1);
            }
        }

        void ThrowOnDisposed()
        {
            if (disposing == null)
            {
                throw new ObjectDisposedException(ToString());
            }
        }

        void ReaderThreadProc()
        {
            byte[] buf = new byte[REPORT_SIZE];

            using (ManualResetEvent asyncRead = new ManualResetEvent(false))
            {
                WaitHandle[] waitHandles = new WaitHandle[2] { asyncRead, disposing };

                while (!disposing.WaitOne(0))
                {
                    uint r;
                    NativeOverlapped overlaped = new NativeOverlapped();
                    overlaped.EventHandle = asyncRead.SafeWaitHandle.DangerousGetHandle();
                    bool ret = NativeMethods.ReadFile(deviceHandle.DangerousGetHandle(), buf, (uint)buf.Length, out r, ref overlaped);
                    if (!ret)
                    {
                        int error = Marshal.GetLastWin32Error();
                        if (error != 997) // 997 = ERROR_IO_PENDING
                        {
                            Debug.WriteLine("ReadFile failed: " + error);
                            continue;
                        }

                        int result = WaitHandle.WaitAny(waitHandles);
                        if (result == 1) // disposing?
                        {
                            NativeMethods.CancelIo(deviceHandle.DangerousGetHandle());
                            asyncRead.WaitOne();
                            continue;
                        }

                        if (!NativeMethods.GetOverlappedResult(deviceHandle.DangerousGetHandle(), ref overlaped, out r, true))
                        {
                            error = Marshal.GetLastWin32Error();
                            Debug.WriteLine("Read GetOverlappedResult failed: " + error);
                            continue;
                        }

                        if (overlaped.InternalLow == new IntPtr(0x00000103)) // 103 = STATUS_PENDING
                        {
                            NativeMethods.CancelIo(deviceHandle.DangerousGetHandle());
                            continue;
                        }
                    }

                    IOEvent e = new IOEvent();
                    e.Timestamp = Clock;
                    e.Length = (int)r;
                    for (int i = 0; i < r; i++) { e.Data.Data[i] = buf[i]; }
                    lock (recieved)
                    {
                        if (recieved.Count > 16) { recieved.Dequeue(); }
                        recieved.Enqueue(e);
                        readCount.Release();
                    }
                }
            }
        }

        void ReadEventDispatchThreadProc()
        {
            byte[] buf = new byte[REPORT_SIZE];
            WaitHandle[] waitHandles = new WaitHandle[2] { readCount, disposing };

            while (!disposing.WaitOne(0))
            {
                int result = WaitHandle.WaitAny(waitHandles);
                if (result == 1) { continue; } // disposing

                IOEvent e;
                lock (recieved)
                {
                    e = recieved.Dequeue();
                }

                for (int i = 0; i < e.Length; i++)
                {
                    buf[i] = e.Data.Data[i];
                }

                if (DeviceReport != null)
                {
                    DeviceReport(this, buf, e.Length, e.Timestamp);
                }
            }
        }

        void WriterThreadProc()
        {
            byte[] buf = new byte[REPORT_SIZE];
            WaitHandle[] waitHandles = new WaitHandle[2] { writeCount, disposing };

            while (!disposing.WaitOne(0))
            {
                int result = WaitHandle.WaitAny(waitHandles);
                if (result == 1) { continue; } // disposing

                IOEvent e;
                lock (sending)
                {
                    e = sending.Dequeue();
                }

                for (int i = 0; i < REPORT_SIZE; i++)
                {
                    buf[i] = e.Data.Data[i];
                }

                NativeOverlapped overlapped = new NativeOverlapped();
                uint ret;
                uint reportLength = !toshibaStackMode ? (uint)e.Length : REPORT_SIZE;
                bool r = NativeMethods.WriteFile(deviceHandle.DangerousGetHandle(), buf, reportLength, out ret, ref overlapped);

                if (!r)
                {
                    int error = Marshal.GetLastWin32Error();

                    if (error == 1784) // 1784 = ERROR_INVALID_USER_BUFFER
                    {
                        toshibaStackMode = true;
                        overlapped = new NativeOverlapped();
                        reportLength = REPORT_SIZE;
                        r = NativeMethods.WriteFile(deviceHandle.DangerousGetHandle(), buf, reportLength, out ret, ref overlapped);
                        error = !r ? Marshal.GetLastWin32Error() : 0;
                    }

                    if (error != 997) // 997 = ERROR_IO_PENDING
                    {
                        Debug.WriteLine("WriteFile failed: " + error);
                        continue;
                    }
                }

                if (!NativeMethods.GetOverlappedResult(deviceHandle.DangerousGetHandle(), ref overlapped, out ret, true))
                {
                    Debug.WriteLine("Write GetOverlappedResult failed: " + Marshal.GetLastWin32Error());
                    continue;
                }
            }
        }

        const int REPORT_SIZE = 22;

        struct IOEvent
        {
            public DateTime Timestamp;
            public int Length;
            public IOBuffer Data;
            [StructLayout(LayoutKind.Explicit)]
            public struct IOBuffer
            {
                [FieldOffset(0)]
                public fixed byte Data[REPORT_SIZE];
            }
        }

        bool toshibaStackMode = false;
        SafeHandle deviceHandle = null;
        ManualResetEvent disposing = new ManualResetEvent(false);
        Queue<IOEvent> sending = new Queue<IOEvent>();
        Queue<IOEvent> recieved = new Queue<IOEvent>();
        Thread readerThread, dispatcherThread, writerThread;
        Semaphore readCount = new Semaphore(0, int.MaxValue);
        Semaphore writeCount = new Semaphore(0, int.MaxValue);
        Stopwatch stopwatch = new Stopwatch();
    }

    class NativeMethods
    {
        [Flags]
        public enum SetupDiGetClassFlags : uint
        {
            Default = 0x00000001,
            Present = 0x00000002,
            AllClasses = 0x00000004,
            Profile = 0x00000008,
            DeviceInterface = 0x00000010,
        }

        [Flags]
        public enum FileAccess : uint
        {
            GenericRead = 0x80000000,
            GenericWrite = 0x40000000,
            GenericExecute = 0x20000000,
            GenericAll = 0x10000000,
        }

        [Flags]
        public enum FileAttributes : uint
        {
            Readonly = 0x00000001,
            Hidden = 0x00000002,
            System = 0x00000004,
            Directory = 0x00000010,
            Archive = 0x00000020,
            Device = 0x00000040,
            Normal = 0x00000080,
            Temporary = 0x00000100,
            SparseFile = 0x00000200,
            ReparsePoint = 0x00000400,
            Compressed = 0x00000800,
            Offline = 0x00001000,
            NotContentIndexed = 0x00002000,
            Encrypted = 0x00004000,
            Write_Through = 0x80000000,
            Overlapped = 0x40000000,
            NoBuffering = 0x20000000,
            RandomAccess = 0x10000000,
            SequentialScan = 0x08000000,
            DeleteOnClose = 0x04000000,
            BackupSemantics = 0x02000000,
            PosixSemantics = 0x01000000,
            OpenReparsePoint = 0x00200000,
            OpenNoRecall = 0x00100000,
            FirstPipeInstance = 0x00080000
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct SP_DEVICE_INTERFACE_DATA
        {
            public int cbSize;
            public Guid InterfaceClassGuid;
            public int Flags;
            public IntPtr RESERVED;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct SP_DEVICE_INTERFACE_DETAIL_DATA
        {
            public UInt32 cbSize;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
            public string DevicePath;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct HIDD_ATTRIBUTES
        {
            public int Size;
            public short VendorID;
            public short ProductID;
            public short VersionNumber;
        }

        [DllImport("hid.dll")]
        public static extern void HidD_GetHidGuid(out Guid gHid);

        [DllImport("hid.dll")]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool HidD_GetAttributes(IntPtr HidDeviceObject, ref HIDD_ATTRIBUTES Attributes);

        [DllImport("hid.dll")]
        [return: MarshalAs(UnmanagedType.U1)]
        public extern static bool HidD_SetOutputReport(IntPtr HidDeviceObject, byte[] lpReportBuffer, uint ReportBufferLength);

        [DllImport("setupapi.dll", SetLastError = true)]
        public static extern IntPtr SetupDiGetClassDevs(ref Guid ClassGuid, IntPtr Enumerator, IntPtr hwndParent, SetupDiGetClassFlags Flags);

        [DllImport("setupapi.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool SetupDiEnumDeviceInterfaces(IntPtr hDevInfo, IntPtr devInfo, ref Guid interfaceClassGuid, int memberIndex, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData);

        [DllImport("setupapi.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr hDevInfo, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, IntPtr deviceInterfaceDetailData, uint deviceInterfaceDetailDataSize, out uint requiredSize, IntPtr deviceInfoData);

        [DllImport("setupapi.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool SetupDiGetDeviceInterfaceDetail(IntPtr hDevInfo, ref SP_DEVICE_INTERFACE_DATA deviceInterfaceData, ref SP_DEVICE_INTERFACE_DETAIL_DATA deviceInterfaceDetailData, uint deviceInterfaceDetailDataSize, out uint requiredSize, IntPtr deviceInfoData);

        [DllImport("setupapi.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool SetupDiDestroyDeviceInfoList(IntPtr hDevInfo);


        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        public static extern SafeFileHandle CreateFile(
            string fileName,
            [MarshalAs(UnmanagedType.U4)] FileAccess fileAccess,
            [MarshalAs(UnmanagedType.U4)] FileShare fileShare,
            IntPtr securityAttributes,
            [MarshalAs(UnmanagedType.U4)] FileMode creationDisposition,
            [MarshalAs(UnmanagedType.U4)] FileAttributes flags,
            IntPtr template);

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool CloseHandle(IntPtr hObject);

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool WriteFile(IntPtr hFile, byte[] lpBuffer,
           uint nNumberOfBytesToWrite, out uint lpNumberOfBytesWritten, ref NativeOverlapped lpOverlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool ReadFile(IntPtr hFile, byte[] lpBuffer,
            uint nNumberOfBytesToRead, out uint lpNumberOfBytesRead, ref NativeOverlapped lpOverlapped);

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool GetOverlappedResult(IntPtr hFile, ref NativeOverlapped lpOverlapped,
           out uint lpNumberOfBytesTransferred, bool bWait);

        [DllImport("kernel32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool CancelIo(IntPtr hFile);
    }

    // TODO: Bluetooth ペアリング
}

