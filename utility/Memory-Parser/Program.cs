using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;

namespace Memory_Parser
{
	class Program
	{
		static private Dictionary<char, int> tokens = new Dictionary<char, int>();
		static private BinaryReader mem_file;
		static private StreamWriter env_data_file;
		static private StreamWriter kin_data_file;

		static private List<byte> data_bytes;

		static void Main(string[] args)
		{
			try
			{
				mem_file = new System.IO.BinaryReader(File.Open("flash_contents.mem", FileMode.Open));
			}
			catch
			{
				System.Console.WriteLine("Could not open flash_contents.mem");
				return;
			}
			try 
			{ 
				env_data_file = new StreamWriter("environmental_data.csv");
				kin_data_file = new StreamWriter("kinematic_data.csv");
			}
			catch
			{
				System.Console.WriteLine("CSV file error");
				return;
			}

			tokens.Add('k', 10); // kinematic data frame
			tokens.Add('e', 18); // kinematic + environmental data frame
			tokens.Add('u', 4); // UV experiment start time
			tokens.Add('v', 4); // UV experiment end time

			data_bytes = new List<byte>(mem_file.ReadBytes((int)mem_file.BaseStream.Length));
			mem_file.Close();
			mem_file = null;

			while (data_bytes.Count > 0)
			{
				while (data_bytes.Count > 0 && !tokens.ContainsKey((char)data_bytes[0]))
				{
					// if invalid token at start of recieved bytes, then remove it
					data_bytes.RemoveAt(0);
				}

				if (data_bytes.Count == 0)
					break;
				char front_char = (char)data_bytes[0];
				if (data_bytes.Count < tokens[front_char] + 2)
					break;

				if ((char)data_bytes[tokens[front_char] + 1] != ',')
				{
					data_bytes.RemoveRange(0, tokens[front_char] + 1);
					continue;
				}

				process_frame(front_char);
				System.Console.WriteLine(data_bytes.Count);
			}
			kin_data_file.Close();
			env_data_file.Close();
			System.Console.WriteLine("Memory parser finished. CSV data files generated.");
		}

		static private void process_frame(char token)
		{
			byte[] bytes = data_bytes.GetRange(0, tokens[token] + 2).ToArray();
			switch (token)
			{
				case 'k':
				case 'e':
					Int32 time = System.BitConverter.ToInt32(bytes, 1);

					UInt16 hi_x = System.BitConverter.ToUInt16(bytes, 5);
					UInt16 hi_y = System.BitConverter.ToUInt16(bytes, 7);
					UInt16 hi_z = System.BitConverter.ToUInt16(bytes, 9);

					kin_data_file.WriteLine(time + "," + hi_x + "," + hi_y + "," + hi_z);

					if (token == 'k')
						break;

					UInt32 pres = System.BitConverter.ToUInt32(bytes, 15);
					Int32 temp = System.BitConverter.ToInt32(bytes, 11);
					env_data_file.WriteLine(time + "," + temp + "," + pres);
					break;
				case 'u':
					break;
				case 'v':
					break;
				default:
					break;
			}
			data_bytes.RemoveRange(0, tokens[token] + 2);
		}

	}
}
