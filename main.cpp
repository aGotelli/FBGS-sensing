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

#include "shape_sensing_interface.h"
#include "illumisense_interface.h"

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001"
#define CALIB_FILE "/home/sven/Downloads/PR2022_14_53_01_S01 - calib.txt"

std::string string_to_hex(const std::string& input)
{
    static const char hex_digits[] = "0123456789abcdef";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input)
    {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}

int main(int argc, char **argv)
{
	
	//IllumiSenseInterface interface(SERVER_ADDRESS,PORT_NUMBER,CALIB_FILE);
	
	//ShapeSensingInterface interface(SERVER_ADDRESS,PORT_NUMBER);
	//
	//if(!interface.connect())
	//	return 0;
	//
	//while(1)
	//{
	//	if(interface.nextSampleReady())
	//	{
	//		ShapeSensingInterface::Sample sample;
	//		
	//		if(interface.readNextSample(sample))
	//		{
	//			std::cout << sample.sample_number << std::endl;
	//			std::cout << sample.num_channels << std::endl;
	//			std::cout << sample.num_sensors << std::endl << std::endl;
	//			
	//			for(int i = 0; i < sample.num_channels; i++)
	//			{
	//				std::cout << sample.channels.at(i).channel_number << std::endl;
	//				std::cout << sample.channels.at(i).num_gratings << std::endl << std::endl;
	//				
	//				std::cout << sample.channels.at(i).error_status(0) << std::endl;
	//				std::cout << sample.channels.at(i).error_status(1) << std::endl;
	//				std::cout << sample.channels.at(i).error_status(2) << std::endl;
	//				std::cout << sample.channels.at(i).error_status(3) << std::endl << std::endl;
	//				
	//				for(int j = 0; j < sample.channels.at(i).num_gratings; j++)
	//				{
	//					std::cout << sample.channels.at(i).peak_wavelengths(j) << " " << sample.channels.at(i).peak_powers(j) << std::endl;
	//				}
	//				std::cout << std::endl;
	//			}
	//			
	//			for(int i = 0; i < sample.num_sensors; i++)
	//			{
	//				std::cout << sample.sensors.at(i).num_curv_points << std::endl;
	//				std::cout << sample.sensors.at(i).num_shape_points << std::endl << std::endl;
	//				
	//				
	//				for(int j = 0; j < sample.sensors.at(i).num_curv_points; j++)
	//				{
	//					std::cout << sample.sensors.at(i).kappa(j) << " " << sample.sensors.at(i).phi(j) << std::endl;
	//				}
	//				std::cout << std::endl;
	//				
	//				
	//				for(int j = 0; j < sample.sensors.at(i).num_shape_points; j++)
	//				{
	//					std::cout << sample.sensors.at(i).shape.row(j) << " " << sample.sensors.at(i).arc_length(j) << std::endl;
	//				}
	//				std::cout << std::endl;
	//				
	//			}
	//		}
	//		
	//		
	//		
	//		
	//	}
	//
	//
	//}
	
	return 1;
	
					
	
}

