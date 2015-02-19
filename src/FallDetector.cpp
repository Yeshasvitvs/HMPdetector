//===============================================================================//
// Name			: FallDetector.cpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.1
// Description	: Fall detector module (on-line only)
//===============================================================================//

#include "HMPdetector/FallDetector.hpp"
#include "HMPdetector/SerialStream.h"

using namespace std;
using namespace boost::posix_time;

// publish the fall detection info on PEIS
//! @param report:	fall report acquired from the waist device
//! @return:		---
void FallDetector::publishFall(string report)
{
	string posture;	// post-fall user posture (Lying/Standing)
	string motion;	// post-fall presence/lack of lower-body motion (Moving/Still)

	if(report.substr(2) == "Fall")
	{
		// report the new fall occurrence on PEIS
		peiskmt_setStringTuple("Fall.Alarm", report.substr(2).c_str());
		cout<<"Alarm: " <<report.substr(2).c_str() <<endl;
	}
	else
	{
		// report posture and user motion after the fall
		istringstream stream(report.substr(2));
		stream >>posture >>motion;
		peiskmt_setStringTuple("Fall.Posture", posture.c_str());
		cout<<"Posture: " <<posture.c_str() <<endl;
		peiskmt_setStringTuple("Fall.Waist_motion", motion.c_str());
		cout<<"Waist_motion: " <<motion.c_str() <<endl;
	}
}

// raise an alarm if the waist device reports a fall
//! @param port:	USB port for data acquisition
//! @return:		---
void FallDetector::standaloneFall(char* port)
{
	string report;	// generic report transmitted by the waist device

	// setup the serial communication (read-only)
	SerialOptions options;
	options.setDevice(port);
	options.setBaudrate(9600);
	options.setTimeout(seconds(0));
	options.setParity(SerialOptions::noparity);
	options.setCsize(8);
	options.setFlowControl(SerialOptions::noflow);
	options.setStopBits(SerialOptions::one);
	SerialStream serial(options);
	serial.exceptions(ios::badbit | ios::failbit);

	// check the sensing device for falls reports
	while(peiskmt_isRunning())
	{
		try
		{
			// read the fall report
			// report in the form: F Fall [Posture][Motion]
			getline(serial,report);
			//DEBUG: cout<<"Message: " <<report <<endl;

			// publish the fall info on PEIS
			publishFall(report);
		}
		catch(TimeoutException&)
		{
			serial.clear();
			cerr<<"Error occurred"<<endl;
		}
	}
}
