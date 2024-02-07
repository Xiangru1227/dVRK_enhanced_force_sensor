#include "cForceSensor.h"
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>
#include <SerialPort.hpp>
#include <iostream>
#include <fstream>
#include "conio.h"

#include <string>

#pragma comment(lib, "ws2_32.lib")
#define SERIALCLASS_H_INCLUDED
#define DEFAULT_PORT "27015"

class Timer {
  public:
    Timer() {
      reset();
    }
    /// reset() makes the timer start over counting from 0.0 seconds.
    void reset() {
      unsigned __int64 pf;
      QueryPerformanceFrequency( (LARGE_INTEGER *)&pf );
      freq_ = 1.0 / (double)pf;
      QueryPerformanceCounter( (LARGE_INTEGER *)&baseTime_ );
    }
    /// seconds() returns the number of seconds (to very high resolution)
    /// elapsed since the timer was last created or reset().
    double seconds() {
      unsigned __int64 val;
      QueryPerformanceCounter( (LARGE_INTEGER *)&val );
      return (val - baseTime_) * freq_;
    }
    /// seconds() returns the number of milliseconds (to very high resolution)
    /// elapsed since the timer was last created or reset().
    double milliseconds() {
      return seconds() * 1000.0;
    }
  private:
    double freq_;
    unsigned __int64 baseTime_;
};

Timer t;
double StartTime = 0;
double ReferenceTime = 0;
double CurrentTime = 0;
double ElapsedTime = 0;
char x[7], y[7], z[7];
char * force_ptr[3] = {x, y, z};

SerialPort* serialport = new SerialPort("COM5");

int main()
{
	double ForceData[3] = {0.00,0.00,0.00};
	bool exit = false;
	int bytesread = 0;
	bool sent = false;
	int buf_idx = 0;
	
	std::string char_string;
	double SensorData[6];

	// Define the force sensor objects from the DAQ
	cForceSensor g_ForceSensor;
	g_ForceSensor.Set_Calibration_File_Loc("FT44298.cal");
	g_ForceSensor.Initialize_Force_Sensor("Dev1/ai0:5");

	Sleep(1000);

	if (serialport->isConnected())
		printf("Serial port connected.\n");

	//preallocate memory we are printing x.xx,x.xx,x.xx,x.xx\n which is 20chars
	char buf[100];
	char outbuf = 'x';

	char kb;
	bool zeroed_flag = false;
	double cumulated[6] = {0,0,0,0,0,0};
	double offset[6] = {0,0,0,0,0,0};
	int cumulate_counter = 0;
	g_ForceSensor.Zero_Force_Sensor();
	StartTime = t.seconds();

	bool new_file = true;
	FILE *f;

	while (true) {

	CurrentTime = t.seconds();
	ElapsedTime = CurrentTime - StartTime;

	if (new_file && serialport->isConnected())
	{
		std::string filename;
		std::cout << "Enter filename: ";
		std::cin >> filename;
		std::cout << "\n Saving to..." << filename << "\n";
		f = fopen(filename.c_str(), "w");
		new_file = false;
	}

	if (CurrentTime - ReferenceTime > 0.01 && ElapsedTime > 1) {
		int i = g_ForceSensor.AcquireFTData();
		g_ForceSensor.GetForceReading(ForceData);
		ReferenceTime = CurrentTime;

		//if sent flag is false we ask the arduino for a new set of readings
		if (sent == false) 
		{
			sent = serialport->writeSerialPort(&outbuf, 1);
		}
		if (sent == true) {
			//if we succeed in sending then we read the serial port 
			bytesread = serialport->readSerialPort(&buf[buf_idx], 35);
			buf_idx += bytesread;
			if (buf_idx > 30)
			{
				sent = false;
				char_string = std::string(buf);
				buf_idx = 0;
				
				// FEATURE TO RECORD USING KEYBOARD PRESSES
				// record on keyboard hit and print the string
				
				// Loop to record and convert the string data into double so that we can perform maths on the quantities.
				int c = 0;
				for (int n = 0; n < 30; n++)
				{
					if (n % 5 == 0)
					{	
 						SensorData[c] = std::stod(char_string.substr(n, 4));
						c++;
					}
				}

				// print data to terminal
				printf("%.3f, ", ElapsedTime);
				for (int n = 0; n < 6; n++)
					printf("%.2f, ", SensorData[n] - offset[n]);
				printf(" F: %+.4f, %+.4f, %+.4f\r", ForceData[0], ForceData[1], ForceData[2]);

				// Comment out the keyboard control to the arduino
				if (_kbhit())
				{
					kb = _getch();

					if (kb == 'z')
					{
						zeroed_flag = false;
						for (int n = 0; n < 6; n++) {
							//offset[n] = cumulated[n] / cumulate_counter;
							offset[n] = SensorData[n];
						}
						printf("\n");
					}

					if (kb == 'x')
					{
						//new_file = true;
						printf("\n");
						std::cout << "Data capture terminated." << std::endl;
						fclose(f);
						break;
					}
					
					//else {
					//	// write the data to the file
					//	fprintf(f, "%.3f, ", ElapsedTime);
					//	fprintf(f, "%.4f,%.4f,%.4f,", ForceData[0], ForceData[1], ForceData[2]);
					//	for (int n = 0; n < 5; n++)
					//		fprintf(f, "%.2f,", SensorData[n]);
					//	fprintf(f, "%.2f\n", SensorData[5]);
					//}
					//printf("\n");
				}
				
				if (!zeroed_flag) { //This is a cumulative counter.
					for (int n = 0; n < 6; n++) {
						cumulated[n] = SensorData[n];
						cumulate_counter++;
					}
				}

				fflush(stdin);

				fprintf(f, "%.3f, ", ElapsedTime);
				fprintf(f, "%.4f, %.4f, %.4f, ", ForceData[0], ForceData[1], ForceData[2]);
				for (int n = 0; n < 5; n++)
					fprintf(f, "%.2f, ", SensorData[n] - offset[n]);
				fprintf(f, "%.2f\n", SensorData[5] - offset[5]);
				/*for (int n = 0;n<30;n++)  
					fprintf(f, "%c", buf[n]);*/
			}
		}
	}
	}
	
	/*
	while (exit==false)
	{	
		int i = g_ForceSensor.AcquireFTData();
		g_ForceSensor.GetForceReading(ForceData);
		CurrentTime = t.seconds();
		//printf("Time: %.3f\r", CurrentTime-StartTime);

		if (CurrentTime - ReferenceTime > 0.03) {

			ReferenceTime = t.seconds();
		
			for (int n = 0; n < 3 ; n++)
			{
				if (ForceData[n]< 0 && ForceData[n] > -10)
				{
					sprintf(force_ptr[n], "%.4f", ForceData[n]);
					//printf("Force: %.4f\r", ForceData[i]);
				}
				else if (ForceData[n] < -10)
				{
					sprintf(force_ptr[n], "%.3f", ForceData[n]);
					//printf("Force: %.3f\r", ForceData[i]);
				}
				else if (ForceData[n] > 0 && ForceData[n] < 10)
				{
					sprintf(force_ptr[n], "%.5f", ForceData[n]);
					//printf("Force: %.5f\r", ForceData[i]);
				}
				else
				{
					sprintf(x, "%.4f", ForceData[n]);
					//printf("Force: %.4f\r", ForceData[i]);
				}

				//printf("Force %d: %f\t Time: %f\r",i, ForceData[i], CurrentTime-StartTime);
				exit = s.send_msg(force_ptr[n]);
			}
		}

	}

	return 0;

	s.kill_server();*/

	printf("Exiting...");
}
