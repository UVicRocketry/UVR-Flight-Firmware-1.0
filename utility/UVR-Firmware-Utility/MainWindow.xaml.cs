using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;

namespace UVR_Firmware_Utility
{

	public partial class MainWindow : Window, INotifyPropertyChanged
	{
		private System.IO.Ports.SerialPort serialPort1;
		private List<Byte> recieved_bytes = new List<Byte>();
		private Dictionary<char, int> tokens = new Dictionary<char, int>();

		private bool writing_to_file = false;

		private System.IO.BinaryWriter mem_file;

		private double hx_curr, hy_curr, hz_curr, hx_zero, hy_zero, hz_one, ref_pres;

		public event PropertyChangedEventHandler PropertyChanged;

		protected void OnPropertyChanged(string propertyName)
		{
			if (PropertyChanged != null)
				PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
		}

		public MainWindow()
		{
			InitializeComponent();
			serialPort1 = new System.IO.Ports.SerialPort();
			
			serialPort1.BaudRate = 115200;
			//set threshold to minimum size of data burst
			serialPort1.ReceivedBytesThreshold = 6;
			serialPort1.ReadBufferSize = 65536;
			
			this.serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serialPort1_DataReceived); 
			comboBoxPorts.ItemsSource = System.IO.Ports.SerialPort.GetPortNames();
			
			tokens.Add('t', 4); // temperature
			tokens.Add('p', 4); // pressure

			tokens.Add('a', 6); // low-g accelerometer
			tokens.Add('q', 8); // quaternion orientation

			tokens.Add('c', 4); // millisecond time elapsed measurement

			tokens.Add('s', 4); // sensor read time
			tokens.Add('m', 4); // bytes written to memory
			tokens.Add('b', 4); // bytes in payload data queue

			tokens.Add('d', 1); // board is now sending logged data to the pc over serial 
			tokens.Add('f', 1); // board has finished sending logged data to the pc over serial
			
			tokens.Add('h', 6); // high-g accelerometer

			tokens.Add('z', 2); // z-axis rest acceleration
			tokens.Add('x', 4); // launch pad reference pressure

			tokens.Add('u', 4); // uv on/off state
		}

		private void buttonConnect_Click(object sender, RoutedEventArgs e)
		{
			try
			{
				serialPort1.Open();
			}
			catch (System.Exception ex) 
			{
				MessageBox.Show("ERROR: Could not open serial port:\n" + ex.Message);
				return;
			}

			comboBoxPorts.IsEnabled = false;
			buttonConnect.IsEnabled = false;
			buttonDisconnect.IsEnabled = true;
			buttonDownload.IsEnabled = true;
			buttonRelPres.IsEnabled = true;
		}

		private void buttonDownload_Click(object sender, RoutedEventArgs e)
		{
			if (!writing_to_file)
			{
				try
				{
					mem_file = new System.IO.BinaryWriter(File.Open("flash_contents.mem",FileMode.Create));
				}
				catch (System.Exception ex)
				{
					MessageBox.Show("ERROR! Could not open csv file:\n" + ex.Message);
					return;
				}
				Bytes_downloaded = 0;
				serialPort1.Write("d");
				buttonDownload.Content = "Stop Download";
				buttonDisconnect.IsEnabled = false;
			}
			else
			{
				end_download();
				serialPort1.Write("a");
			}
		}

		private void end_download()
		{
			serialPort1.DataReceived -= serialPort1_DataReceived;

			Bytes_downloaded += recieved_bytes.Count;

			for (int i = 0; i < recieved_bytes.Count; i++)
			{
				mem_file.Write(recieved_bytes[i]);

				//csv_file.Write(recieved_bytes[i]);
			}
			recieved_bytes.Clear();
			serialPort1.DiscardInBuffer();
			serialPort1.ReceivedBytesThreshold = 6;

			writing_to_file = false;

			mem_file.Close();
			mem_file = null;

			buttonDownload.Content = "Start Download";
			buttonDisconnect.IsEnabled = true;
			serialPort1.DataReceived += serialPort1_DataReceived;
		}

		
		private void buttonCalH_Click(object sender, RoutedEventArgs e)
		{
			hx_zero = hx_curr;
			hy_zero = hy_curr;
			hz_one = hz_curr;
		}

		private void buttonRelPres_Click(object sender, RoutedEventArgs e)
		{
			ref_pres = Pres;
		}

		private void comboBoxPorts_SelectionChanged(object sender, SelectionChangedEventArgs e)
		{
			if (comboBoxPorts.SelectedItem.ToString() != "")
			{
				buttonConnect.IsEnabled = true;
				serialPort1.PortName = comboBoxPorts.SelectedItem.ToString();
			}
			else
				buttonConnect.IsEnabled = false;
		}

		private void buttonDisconnect_Click(object sender, RoutedEventArgs e)
		{
			if (!serialPort1.IsOpen)
				return;
			serialPort1.Close();
			comboBoxPorts.IsEnabled = true;
			buttonConnect.IsEnabled = true;
			buttonDisconnect.IsEnabled = false;
			buttonDownload.IsEnabled = false;
			buttonRelPres.IsEnabled = false;
		}

		// parse data values from recieved frame
		private void process_frame(char token)
		{
			byte[] bytes = recieved_bytes.ToArray();
			switch (token)
			{
				case 'c':
					Time_between_measurements = Time_elapsed;
					Time_elapsed = System.BitConverter.ToUInt32(bytes, 1);
					Time_between_measurements = Time_elapsed - Time_between_measurements;
					break;
				case 'a':
					A_x = (double)System.BitConverter.ToInt16(bytes, 1) / 100.0;
					A_y = (double)System.BitConverter.ToInt16(bytes, 3) / 100.0;
					A_z = (double)System.BitConverter.ToInt16(bytes, 5) / 100.0;
					A_net = Math.Sqrt(Math.Pow(A_x, 2.0) + Math.Pow(A_y, 2.0) + Math.Pow(A_z, 2.0));
					break;
				case 'q':
					Q_w = System.BitConverter.ToInt16(bytes, 1);
					Q_x = System.BitConverter.ToInt16(bytes, 3);
					Q_y = System.BitConverter.ToInt16(bytes, 5);
					Q_z = System.BitConverter.ToInt16(bytes, 7);

					// process quaternion into euler angles and update displayed euler values

					// normalize the quaternion
					double magnitude = Math.Sqrt(Q_w * Q_w + Q_x * Q_x + Q_y * Q_y + Q_z * Q_z);
					double qx = Q_x / magnitude;
					double qy = Q_y / magnitude;
					double qz = Q_z / magnitude;
					double qw = Q_w / magnitude;

					double ysqr = qy * qy;

					// roll (x-axis rotation)
					double t0 = +2.0 * (qw * qx + qy * qz);
					double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
					Roll = Math.Atan2(t0, t1) * (180.0 / Math.PI);

					// pitch (y-axis rotation)
					double t2 = +2.0 * (qw * qy - qz * qx);
					t2 = t2 > 1.0 ? 1.0 : t2;
					t2 = t2 < -1.0 ? -1.0 : t2;
					Pitch = Math.Asin(t2) * (180.0 / Math.PI);

					// yaw (z-axis rotation)
					double t3 = +2.0 * (qw * qz + qx * qy);
					double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
					Yaw = Math.Atan2(t3, t4) * (180.0 / Math.PI);

					break;
				case 't':
					Temp = (double)System.BitConverter.ToInt32(bytes, 1) / 100.0;
					break;
				case 'p':
					Pres = (double)System.BitConverter.ToUInt32(bytes, 1);
					if (ref_pres == 0.0)
						ref_pres = Pres;
					Rel_Alt = 44330.0 * (1.0 - Math.Pow(Pres / ref_pres, 0.1903));
					break;

				case 'h':
					hx_curr = System.BitConverter.ToUInt16(bytes, 1);
					hy_curr = System.BitConverter.ToUInt16(bytes, 3);
					hz_curr = System.BitConverter.ToUInt16(bytes, 5);

					
					const double ratio = 400.0 / 65535.0;
					const double half = 65535.0 / 2.0;

					H_x = (double)(hx_curr - half) * ratio;
					H_y = (double)(hy_curr - half) * ratio;
					H_z = (double)(hz_curr - half) * ratio;

					break;
				case 'z':
					h_lpz = System.BitConverter.ToUInt16(bytes, 1);
					H_LPZ = (double)(h_lpz - half) * ratio;
					break;
				case 'x':
					LP_Pres = (double)System.BitConverter.ToUInt32(bytes, 1);
					break;
				case 'm':
					Bytes_written = System.BitConverter.ToInt32(bytes, 1);
					break;
				case 'b':
					DQ_Size = System.BitConverter.ToUInt32(bytes, 1);
					break;
				case 's':
					Sensor_delay = System.BitConverter.ToUInt32(bytes, 1);
					break;
				case 'u':
					int diode_state = System.BitConverter.ToInt32(bytes, 1);
					Diode_on = (diode_state != 0);
					break;
				case 'd':
					if (!writing_to_file)
					{
						// board will wait 10ms before transmitting memory contents
						serialPort1.DiscardInBuffer();
						serialPort1.ReceivedBytesThreshold = 256;
						writing_to_file = true;
						
						// immediately clear recieved bytes buffer and exit the function in the special case
						// that the pc requests a memory download
						recieved_bytes.Clear();
						return;
					}
					break;
				case 'f':
					if (writing_to_file)
					{
						end_download();
					}
					break;
				default:
					break;
			}
			// will raise an exception if invalid token used
			recieved_bytes.RemoveRange(0, tokens[token] + 2);
		}

		private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
		{
			try
			{
				Btr = serialPort1.BytesToRead;
				// retrieve bytes from serial read buffer
				while (serialPort1.BytesToRead > 0)
					recieved_bytes.Add((Byte)serialPort1.ReadByte());
			}
			// if serial port is closed while trying to read from it, then abort
			catch (System.InvalidOperationException)
			{
				return;
			}

			if (writing_to_file)
			{
				Bytes_downloaded += recieved_bytes.Count;
				for (int i = 0; i < recieved_bytes.Count; i++)
				{
					mem_file.Write(recieved_bytes[i]);
					//csv_file.Write(recieved_bytes[i]);
				}
				recieved_bytes.Clear();
				return;
			}

			while (true)
			{
				// scan recieved bytes for a valid data token
				while (recieved_bytes.Count > 0 &&
					!tokens.ContainsKey((char)recieved_bytes[0]))
				{
					// if invalid token at start of recieved bytes, then remove it
					recieved_bytes.RemoveAt(0);
				}

				// if not enough bytes to read for full data burst, then exit and 
				// wait for next transmission
				if (recieved_bytes.Count == 0)
					break;
				char front_char = (char)recieved_bytes[0];
				if (recieved_bytes.Count < tokens[front_char] + 2)
				{
					break;
				}

				// if there is a data frame error, then remove the invalid bytes
				// and restart token scanner
				if ((char)recieved_bytes[tokens[front_char] + 1] != ',')
				{
					recieved_bytes.RemoveRange(0, tokens[front_char] + 1);
					Err_count++;
					continue;
				}

				// process the (hopefully) complete data frame
				process_frame(front_char);
			}
		}
	}
}
