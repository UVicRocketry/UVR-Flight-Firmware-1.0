using System;

namespace UVR_Firmware_Utility
{
	public partial class MainWindow
	{
		public readonly int MEMORY_MAX_BYTES = 16777216;

		#region TextBlockData
		#region BNO055
		public double yaw, pitch, roll;
		public double Yaw
		{
			get
			{ return yaw; }

			set
			{
				if (yaw != value)
				{
					yaw = value;
					OnPropertyChanged("yaw");
				}
			}
		}
		public double Pitch
		{
			get
			{ return pitch; }

			set
			{
				if (pitch != value)
				{
					pitch = value;
					OnPropertyChanged("pitch");
				}
			}
		}
		public double Roll
		{
			get
			{ return roll; }

			set
			{
				if (roll != value)
				{
					roll = value;
					OnPropertyChanged("roll");
				}
			}
		}

		public Int16 q_w, q_x, q_y, q_z;
		public Int16 Q_w
		{
			get { return q_w; }

			set
			{
				if (q_w != value)
				{
					q_w = value;
					OnPropertyChanged("q_w");
				}
			}
		}
		public Int16 Q_x
		{
			get { return q_x; }

			set
			{
				if (q_x != value)
				{
					q_x = value;
					OnPropertyChanged("q_x");
				}
			}
		}
		public Int16 Q_y
		{
			get { return q_y; }

			set
			{
				if (q_y != value)
				{
					q_y = value;
					OnPropertyChanged("q_y");
				}
			}
		}
		public Int16 Q_z
		{
			get { return q_z; }

			set
			{
				if (q_z != value)
				{
					q_z = value;
					OnPropertyChanged("q_z");
				}
			}
		}

		public double a_x, a_y, a_z, a_net;
		public double A_x
		{
			get { return a_x; }

			set
			{
				if (a_x != value)
				{
					a_x = value;
					OnPropertyChanged("a_x");
				}
			}
		}
		public double A_y
		{
			get { return a_y; }

			set
			{
				if (a_y != value)
				{
					a_y = value;
					OnPropertyChanged("a_y");
				}
			}
		}
		public double A_z
		{
			get { return a_z; }

			set
			{
				if (a_z != value)
				{
					a_z = value;
					OnPropertyChanged("a_z");
				}
			}
		}
		public double A_net
		{
			get { return a_net; }

			set
			{
				if (a_net != value)
				{
					a_net = value;
					OnPropertyChanged("a_net");
				}
			}
		}
		#endregion
		#region ADXL377
		public double h_x, h_y, h_z, h_lpz;
		public double H_x
		{
			get { return h_x; }

			set
			{
				if (h_x != value)
				{
					h_x = value;
					OnPropertyChanged("h_x");
				}
			}
		}
		public double H_y
		{
			get { return h_y; }

			set
			{
				if (h_y != value)
				{
					h_y = value;
					OnPropertyChanged("h_y");
				}
			}
		}
		public double H_z
		{
			get { return h_z; }

			set
			{
				if (h_z != value)
				{
					h_z = value;
					OnPropertyChanged("h_z");
				}
			}
		}
		public double H_LPZ
		{
			get { return h_lpz; }

			set
			{
				if (h_lpz != value)
				{
					h_lpz = value;
					OnPropertyChanged("h_lpz");
				}
			}
		}
		#endregion
		#region BMP280
		public double temp, pres, rel_alt, lp_pres;
		public double Temp
		{
			get { return temp; }
			set
			{
				if (temp != value)
				{
					temp = value;
					OnPropertyChanged("temp");
				}
			}
		}
		public double Pres
		{
			get { return pres; }
			set
			{
				if (pres != value)
				{
					pres = value;
					OnPropertyChanged("pres");
				}
			}
		}
		public double LP_Pres
		{
			get { return lp_pres; }
			set
			{
				if (lp_pres != value)
				{
					lp_pres = value;
					OnPropertyChanged("lp_pres");
				}
			}
		}
		public double Rel_Alt
		{
			get { return rel_alt; }
			set
			{
				if (rel_alt != value)
				{
					rel_alt = value;
					OnPropertyChanged("rel_alt");
				}
			}
		}
		#endregion
		#region W25Q128FV
		public int bytes_written, bytes_downloaded; 
		public uint dq_size;
		public int Bytes_written
		{
			get { return bytes_written; }
			set
			{
				if (bytes_written != value)
				{
					bytes_written = value;
					OnPropertyChanged("bytes_written");
				}
			}
		}
		public int Bytes_downloaded
		{
			get { return bytes_downloaded; }
			set
			{
				if (bytes_downloaded != value)
				{
					bytes_downloaded = value;
					OnPropertyChanged("bytes_downloaded");
				}
			}
		}
		public uint DQ_Size
		{
			get { return dq_size; }
			set
			{
				if (dq_size != value)
				{
					dq_size = value;
					OnPropertyChanged("dq_size");
				}
			}

		}
		#endregion
		#region COM
		public int err_count, btr;
		public int Err_count
		{
			get { return err_count; }
			set
			{
				if (err_count != value)
				{
					err_count = value;
					OnPropertyChanged("err_count");
				}
			}
		}

		public int Btr
		{
			get { return btr; }
			set
			{
				if (btr != value)
				{
					btr = value;
					OnPropertyChanged("btr");
				}
			}
		}

		#endregion
		#region Time
		public uint time_elapsed, time_between_measurements, sensor_delay;
		public uint Time_elapsed
		{
			get { return time_elapsed; }
			set
			{
				if (time_elapsed != value)
				{
					time_elapsed = value;
					OnPropertyChanged("time_elapsed");
				}
			}
		}
		public uint Time_between_measurements
		{
			get { return time_between_measurements; }
			set
			{
				if (time_between_measurements != value)
				{
					time_between_measurements = value;
					OnPropertyChanged("time_between_measurements");
				}
			}
		}
		public uint Sensor_delay
		{
			get { return sensor_delay; }
			set
			{
				if (sensor_delay != value)
				{
					sensor_delay = value;
					OnPropertyChanged("sensor_delay");
				}
			}
		}
		#endregion
		#region Diodes
		public bool diode_on;
		public bool Diode_on
		{
			get { return diode_on; }
			set
			{
				if (diode_on != value)
				{
					diode_on = value;
					OnPropertyChanged("diode_on");
				}
			}
		}
		#endregion

		#endregion
	}

}