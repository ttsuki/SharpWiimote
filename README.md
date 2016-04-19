# SharpWiimote
Wii リモコンをC#から。

~~~cs
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
                    remoxon.SetLED(2 | 4);
                    Console.ReadLine();
                }
            }
        }
    }
~~~
