/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

//includes
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>


#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "fbgs-sensing/shape_sensing_interface.h"
#include "fbgs-sensing/illumisense_interface.h"

#include <algorithm>

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001" //Default port number for Shape Sensing
//#define PORT_NUMBER "2055" //Default port number for Illumisense
//#define CALIB_FILE "/home/sven/Downloads/PR2022_14_53_01_S01 - calib.txt"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;



void processResult(const std::vector<unsigned int> &sample_numbers)
{
    std::vector<unsigned int> steps;
    for(unsigned int i=1; i<sample_numbers.size(); i++){
        unsigned int diff = sample_numbers[i] - sample_numbers[i-1];
        steps.push_back(diff);
        //std::cout << "diff : " << diff << "\n";
    }

    steps.erase(steps.begin(), steps.begin()+3);

    std::vector<unsigned int> step_jumps;

    std::copy_if(steps.begin(), steps.end(),
                 std::back_inserter(step_jumps), [&](unsigned int diff){ return diff>1?true:false;});


    unsigned int total = std::accumulate(step_jumps.begin(), step_jumps.end(), 0);
    const auto min_max = std::minmax_element(step_jumps.begin(), step_jumps.end());



    double percentage = 100.0*static_cast<double>(step_jumps.size())/static_cast<double>(steps.size());
    std::cout << "step_jump : " << step_jumps.size() << " On a total of " << steps.size() << " steps -> " << percentage << "%\n";
    std::cout << "Everage step : " << total/step_jumps.size() << " min : " << *min_max.first.base() << " max : " << *min_max.second.base() << "\n\n";
    std::cout.flush();

}

int main(int argc, char **argv)
{
	

	//Example code for Shape Sensing Interface
	
	ShapeSensingInterface interface(SERVER_ADDRESS,PORT_NUMBER);
	
	if(!interface.connect())
        return 0;

    std::cout << "connected !" << std::endl;



    ShapeSensingInterface::Sample sample;
    interface.initialiseMemory(sample);
    std::cout << "ok memory !" << std::endl;


    const double frequency = 150; //Hz
    const auto dt_ms = std::chrono::milliseconds(static_cast<unsigned int>((1.0/frequency)*1000));

    const unsigned int max_count = 2000;
    std::vector<unsigned int> sample_numbers(max_count);
    unsigned int count = 0;
    while(count<max_count)
	{
		if(interface.nextSampleReady())
		{

			
			if(interface.readNextSample(sample))
			{
                sample_numbers[count] = sample.sample_number;
                count++;

			}
			
			
			
			
		}

        std::this_thread::sleep_for(dt_ms );
	
	
	}


    processResult(sample_numbers);


    std::cout << "\n\nNow new version \n\n";



    std::vector<boost::asio::streambuf> data_buffers(max_count);
    count = 0;
    while(count<max_count)
    {
        if(interface.nextSampleReady())
        {
            if(interface.getData(data_buffers[count]))
            {
                count++;
            }

        }

        std::this_thread::sleep_for(dt_ms );

    }


    sample_numbers.clear();
    std::for_each(data_buffers.begin(),
                  data_buffers.end(),
                  [&](boost::asio::streambuf &data){
                      const auto sample = interface.processData(data);
                      sample_numbers.push_back( sample.sample_number );
                  });




    processResult(sample_numbers);



	
	return 1;
	
					
	
}

