//===============================================================================//
// Name			: HMPdetector.cpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.1
// Description	: Human Motion Primitives and Fall detection system
//===============================================================================//

#include <getopt.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Char.h>
#include "HMPdetector/creator.hpp"
#include "HMPdetector/classifier.hpp"
#include "HMPdetector/reasoner.hpp"
#include "HMPdetector/FallDetector.hpp"
#include "HMPdetector/SerialStream.h"

using namespace boost::posix_time;
using namespace ros;

//Variables for ROS implementation
std_msgs::Char letter_detected;
ros::Publisher gesture_pub;

//void gesture_receive(int);

void mySigintHandler(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}



//! perform on-line full analysis of a data stream
void completeHMPdetector(char* port, Reasoner oneR, FallDetector oneF)
{
	string raw_data;			// current raw data acquired via USB
	int nS = 0;					// number of acquired samples
	mat actsample;				// current sample in matrix format
	int ax, ay, az;				// accelerometer current sample components
	int gx, gy, gz;				// gyroscope current sample components
	char dev;					// flag --> device type
	string motion;				// flag --> level of motion at the wrist
	vector<float> poss;			// models possibilities
	vector<float> past_poss;	// models previous possibilities
	

	string waste = " ";

	// instantiate and initialize a Classifier
	string dF = oneR.datasetFolder.substr(0,oneR.datasetFolder.length()-1);
	dF = dF.substr(9);
	Classifier hC(dF);
	mat window = zeros<mat>(hC.window_size, 3);
	mat gravity = zeros<mat>(hC.window_size, 3);
	mat body = zeros<mat>(hC.window_size, 3);

	// initialize the possibilities and past possibilities
	for(int i = 0; i < oneR.nbM; i++)
	{
		poss.push_back(0);
		past_poss.push_back(0);
	}

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

	// classify the stream of raw acceleration & gyroscope data
	while(peiskmt_isRunning())
	{
		try
		{
			// acquire the data received by the XBee
			getline(serial,raw_data);
			switch (raw_data[0])
			{
				// call the FallDetector if the raw_data begin with "F"
				// --> data format: F Fall [Posture] [Motion]
				case 'F':
					oneF.publishFall(raw_data);
					break;
				// call the Reasoner if the raw_data begin with "H"
				// --> data format: H [ax] [ay] [ax] [gx] [gy] [gz] [Motion]
				case 'H':
					istringstream stream(raw_data);
					stream >>dev >>ax >> ay >>az >>gx >>gy >>gz >>motion;
					actsample <<ax <<ay <<az;

					// update the window of samples to be analyzed
					hC.createWindow(actsample, window, hC.window_size, nS);
					if (nS >= hC.window_size)
					{
						// analyze the window and compute the models possibilities
						for(int i = 0; i < oneR.nbM; i++)
							past_poss[i] = poss[i];
						hC.analyzeWindow(window, gravity, body);
						hC.compareAll(gravity, body, poss);

						// publish the dynamic tuples
						hC.publishDynamic(poss);

						// extract/update activation intervals for each activity
						for(int i = 0; i < oneR.nbM; i++)
							oneR.updateInterval(i,nS,poss[i],past_poss[i],1,waste);
					}
					break;
			}
		}
		catch(TimeoutException&)
		{
			serial.clear();
			cerr<<"Timeout occurred"<<endl;
		}
	}
}

//! Program help 
void print_help()
{
	cout<<endl;
	cout<<"\t\t -------------- HMP DETECTOR --------------" <<endl;
	cout<<"Detailed help is in the Documentation inside the docs folder." <<endl;
	cout<<"Typical calls use the following instructions:" <<endl;
	cout<<"01) -h --help \t\t\t   : program help." <<endl;
	cout<<"02) -m --model [dataset] \t   : [dataset] models creation." <<endl;
	cout<<"03) -l --load [dataset] \t   : load models in [dataset]." <<endl;
	cout<<"04) -v --validate [model] [set] [n]:"
		<<" validate [model] with [n] trials of [set]." <<endl;
	cout<<"05) -t --test [trial] \t\t   :"
		<<" off-line classification of [trial]." <<endl;
	cout<<"06) -c --classify [port] \t   :"
		<<" on-line classification of [port] stream." <<endl;
	cout<<"07) -r --reason [path] [possFile]  :"
		<<" off-line reasoning on [path]/[possFile]." <<endl;
	cout<<"08) -i --interval [port] \t   :"
		<<" on-line reasoning on [port] stream." <<endl;
	cout<<"09) -f --fall [port] \t\t   :"
		<<" on-line fall detection in [port] stream." <<endl;
	cout<<"10) -u --ultimate [port] \t   :"
		<<" on-line full analysis of [port] stream." <<endl;

	cout<<endl;
	cout<<"Functions calls examples:" <<endl;
	cout<<"01)   ./HMPdetector -h" <<endl;
	cout<<"02.1) ./HMPdetector -m" <<endl;
	cout<<"02.2) ./HMPdetector -m Ovada" <<endl;
	cout<<"03)   ./HMPdetector -l Ovada" <<endl;
	cout<<"04)   ./HMPdetector -v climb Ovada 6" <<endl;
	cout<<"05)   ./HMPdetector -t drink_drink_stand_sit_drink.txt" <<endl;
	cout<<"06)   ./HMPdetector -c /dev/ttyUSB0" <<endl;
	cout<<"07)   ./HMPdetector -r "
		<<"./Results/longTest/ res_drink_drink_stand_sit_drink.txt" <<endl;
	cout<<"08)   ./HMPdetector -i /dev/ttyUSB0" <<endl;
	cout<<"09)   ./HMPdetector -f /dev/ttyUSB0" <<endl;
	cout<<"10)   ./HMPdetector -u /dev/ttyUSB0" <<endl;

	cout<<endl;
	cout<<"Enjoy!"<<endl;

	cout<<endl;
}

void gesture_receive(char new_value){
	
	//cout<<"new_value : "<<new_value<<endl;
	letter_detected.data=new_value;
	//ROS code for publishing the detected gesture
	//cout<<"Letter Received : "<<letter_detected.data<<endl;
	gesture_pub.publish(letter_detected);

}

int main(int argc, char* argv[])
//int main(int argc, char** argv)
{

	//ROS functionality 
	cout<<"ArgC Value : "<<argc<<endl;
	//cout<<*argv[0]<<" "<<*argv[1]<<" "<<*argv[2]<<endl;
	//ROS Initialization
	ros::init(argc,argv,"HMPdetector");
        ROS_INFO("HMP Node creation");
	ros::NodeHandle n_hmp;


	//Publishing
	gesture_pub=n_hmp.advertise<std_msgs::Char>("/gesture_detected",1);



	// instantiate & initialize the PEIS component
	peiskmt_initialize(&argc, argv);
	// retrieve componentID from the component
	int componentID = peiskmt_peisid();
	printf("componentID: %d\n", componentID);

	// instantiate & initialize the HMPdetector components
	string dF = "Letters";
	cout<<"Initializing HMPdetector..."<<endl;	

	Creator one_creator(dF);
	Classifier one_classifier(dF); //ROS process dying HERE -- Fixed this by correcting the path config files
	//DEBUG: one_classifier.printSetInfo();
	Reasoner one_reasoner(dF);
	//DEBUG: one_reasoner.printSetStatus();
	cout<<endl <<"DONE" <<endl;
	cout<<"Dataset folder: " <<one_reasoner.datasetFolder <<endl;
	FallDetector one_falldetector;

	//// models - Orebro
	//model Climb("Climb", 5, 13, 16);		// format: ax ay az gx gy gz
	////model Drink("Drink", 5, 8, 11);		// format: ax ay az
	//model PickUpDrink("PickUpDrink", 5, 8, 11);	// format: ax ay az
	//model PutDownDrink("PutDownDrink", 5, 8, 11);	// format: ax ay az
	////model Pour("Pour", 5, 8, 11);			// format: ax ay az gx gy gz
	//model PickUpPour("PickUpPour", 5, 8, 11);	// format: ax ay az gx gy gz
	//model PutDownPour("PutDownPour", 5, 8, 11);	// format: ax ay az gx gy gz
	//model Sit("Sit", 5, 6, 6);			// format: ax ay az gx gy gz
	//model Stand("Stand", 5, 7, 6);			// format: ax ay az gx gy gz
	//model Walk("Walk", 5, 13, 21);			// format: ax ay az gx gy gz

	// available options (short-form)
	const char *short_options = "v:r:ufictl:mh";
	// available options (long-form)
	static struct option long_options[] = 
	{
		{"validate", required_argument, 0, 'v'},
		{"reason", required_argument, 0, 'r'},
		{"ultimate", required_argument, 0, 'u'},
		{"fall", required_argument, 0, 'f'},
		{"interval", required_argument, 0, 'i'},
		{"classify", required_argument, 0, 'c'},
		{"test", required_argument, 0, 't'},
		{"load", required_argument, 0, 'l'},
		{"model", optional_argument, 0, 'm'},
		{"help", no_argument, 0, 'h'},
		{0, 0, 0, 0} //required line
	};

	// retrieve & execute the chosen option  
	char c;
	c = getopt_long(argc, argv, short_options, long_options, NULL);

	//while(ros::ok()){
	
	do 
	{
		
		switch (c)
		{
			case 'h': //ROS implementation works
				print_help();
				return EXIT_SUCCESS;
				break;
			case 'm': //ROS implementation works
				if(argv[2])
					one_creator.setDatasetFolder(argv[2]);				
				one_creator.generateAllModels();
				cout<<"modelling dataset in: "<<one_creator.datasetFolder <<endl;
				return EXIT_SUCCESS;
				break;
			case 'l': //ROS implementation works				
				one_classifier.buildSet(argv[2]);
				cout<<"loaded dataset of: "<<one_classifier.datasetFolder <<endl;
				return EXIT_SUCCESS;
				break;
			case 'v': //ROS implementation works
				one_classifier.validateModel(argv[2], argv[3], atoi(argv[4]));
				//one_classifier.printSetInfo(); //Debug Code
				cout<<"results in: ./Results/" <<argv[3] <<"/" <<endl;
				return EXIT_SUCCESS;
				break;
			case 't': //ROS implementation not done
				one_classifier.longTest(argv[2]);
				cout<<"results in: ./Results/longTest/" <<endl;
				return EXIT_SUCCESS;
				break;
			case 'c': //ROS implementation works
				cout<<"use 'tupleview' to monitor the system" <<endl;
				one_classifier.onlineTest(argv[2]);
				//signal(SIGINT, mySigintHandler);
				return EXIT_SUCCESS;
				break;
			case 'r': //ROS implementation not done
				one_reasoner.offlineReasoner(argv[2], argv[3]);
				cout<<"results in: " <<argv[2] <<endl;
				return EXIT_SUCCESS;
				break;
			case 'i': //ROS implementation not done
				cout<<"use 'tupleview' to monitor the system" <<endl;
				one_reasoner.onlineReasoner(argv[2]);
				return EXIT_SUCCESS;
				break;
			case 'f': //ROS implementation not done
				cout<<"use 'tupleview' to monitor the system" <<endl;
				one_falldetector.standaloneFall(argv[2]);
				return EXIT_SUCCESS;
				break;
			case 'u': //ROS implementation not done
				cout<<"use 'tupleview' to monitor the system" <<endl;
				completeHMPdetector(argv[2], one_reasoner, one_falldetector);
				return EXIT_SUCCESS;
				break;
			default:
				print_help();
				return EXIT_SUCCESS;
				break;
		}
	}while (c != -1);


		
		//}//End of ROS Ok while loop

	
	return 0;
}
	
