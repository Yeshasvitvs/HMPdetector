//===============================================================================//
// Name			: FallDetector.hpp
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version		: 1.1
// Description	: Fall detector module (on-line only)
//===============================================================================//

#include <iostream>

#ifdef __cplusplus
extern "C"
{
#endif	
	#include <peiskernel/peiskernel_mt.h>
#ifdef __cplusplus
}
#endif

using namespace std;

#ifndef FALLDETECTOR_HPP_
#define FALLDETECTOR_HPP_

//! definition of the class "FallDetector"
class FallDetector
{
	public:
		//! constructor
		FallDetector()
		{
			//DEBUG: cout<<endl <<"Creating FallDetector object" <<endl;
		}

		// publish the fall detection info on PEIS
		void publishFall(string report);

		// raise an alarm if the waist device reports a fall
		void standaloneFall(char* port);

		//! destructor
		~FallDetector()
		{
			//DEBUG: cout<<endl <<"Destroying FallDetector object" <<endl;
		}
};

#endif
