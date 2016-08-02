# SharpWiimote

## Access your Wii Remotes (Wiimotes / RVL-CNT-01-TR) from C#.
C#でWiiリモコンで遊ぶ。

### Supported stacks:
* Microsoft Bluetooth stack (Windows 8 or later)
* Toshiba Bluetooth stack (Windows 7 or earlier)

### Supported Inputs: (入力)
* Buttons / ボタン (ON/OFF)
* Accelerometers / 加速度センサ (X,Y,Z)
* IR Camera / 赤外線センサ (5x X,Y)　(with Wii Sensor Bar)
* Gyrometer / ジャイロセンサ (Y,P,R) (with WiiMotionPlus)
* Battery left / バッテリ残量

### Supported Outputs: (出力)
* Rumbling / 振動 (ON/OFF)
* LED Indicators / LED (PLAYER 1~4)

## License
MIT

## Usage
1. Connect Wii Remote to Windows by Bluetooth.
 1. Press *SYNC* button of the Wii Remote.
 1. Search Bluetooth devices in Windows and pair with empty PIN.
1. Run the code below.
~~~cs
    public static class WiimoteDeviceTest
    {
        public static void Main()
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
                    remocon.SetLED(2 | 4);
                    Console.ReadLine(); // Exit if enter key pressed.
                }
            }
        }
    }
~~~
