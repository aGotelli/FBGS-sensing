/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

//includes
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>


#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "shape_sensing_interface.h"

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001"

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
	//		interface.readNextSample(sample);
	//	}
	//
	//
	//}
	//
	//return 1;
	
	std::ifstream in("/home/sven/Downloads/FBGS_sample_output.txt");
	std::istream is2(in.rdbuf());
	
	std::string data_string;
	
	ShapeSensingInterface::Sample sample;
	
	//Skip first two entries (date and time)
	getline(is2,data_string, '\n'); 
	getline(is2,data_string, '\n'); 
	
	//Next string is the sample number
	getline(is2,data_string, '\n'); 
	sample.sample_number = std::stoi(data_string);
	
	//Next string is number of channels
	getline(is2,data_string, '\n'); 
	sample.num_channels = std::stoi(data_string);
	
	//Now we run through all channels
	for(int i = 0; i < sample.num_channels; i++)
	{
		ShapeSensingInterface::Sample::Channel channel;
		
		//Next string is channel number
		getline(is2,data_string, '\n'); 
		channel.channel_number = std::stoi(data_string);
		
		//Next string is number of gratings
		getline(is2,data_string, '\n'); 
		channel.num_gratings = std::stoi(data_string);
		
		//Next is error status
		getline(is2,data_string, '\n'); 
		channel.error_status(0) = std::stoi(data_string);
		getline(is2,data_string, '\n'); 
		channel.error_status(1) = std::stoi(data_string);
		getline(is2,data_string, '\n'); 
		channel.error_status(2) = std::stoi(data_string);
		getline(is2,data_string, '\n'); 
		channel.error_status(3) = std::stoi(data_string);
		
		//Next is peak wavelengths
		channel.peak_wavelengths.resize(channel.num_gratings);
		for(int j = 0; j < channel.num_gratings; j++)
		{
			getline(is2,data_string, '\n'); 
			channel.peak_wavelengths(j) = std::stod(data_string);
		}
		
		//Next is peak powers
		channel.peak_powers.resize(channel.num_gratings);
		for(int j = 0; j < channel.num_gratings; j++)
		{
			getline(is2,data_string, '\n'); 
			channel.peak_powers(j) = std::stod(data_string);
		}	
		
		sample.channels.push_back(channel);
		
	}
	
	
	//Now run through the file to the end
	getline(is2,data_string, '\n'); 
	int num_sensors = 0;
	while(data_string == "Curvature [1/cm]")
	{
		ShapeSensingInterface::Sample::Sensor sensor;
		sensor.num_curv_points = sample.channels.at(0 + 4*num_sensors).num_gratings;
		
		//Save kappa (curvature) values
		sensor.kappa.resize(sensor.num_curv_points);
		for(int j = 0; j < sensor.num_curv_points; j++)
		{
			getline(is2,data_string, '\n'); 
			sensor.kappa(j) = 100*std::stod(data_string); //convert 1/cm to 1/m	
		}	
		
		//Next entry is text field (skip)
		getline(is2,data_string, '\n'); 
		
		//Save phi (curvature angle) values in rad
		sensor.phi.resize(sensor.num_curv_points);
		for(int j = 0; j < sensor.num_curv_points; j++)
		{
			getline(is2,data_string, '\n'); 
			sensor.phi(j) = std::stod(data_string);
		}
		
		//Next entry is text field (skip)
		getline(is2,data_string, '\n'); 	
		
		//Next entry is number of shape points
		getline(is2,data_string, '\n'); 	
		sensor.num_shape_points = std::stoi(data_string);
		
		sensor.shape.resize(sensor.num_shape_points,3);
		sensor.arc_length.resize(sensor.num_shape_points);
		
		//Save all x values and arclength values
		for(int j = 0; j < sensor.num_shape_points; j++)
		{
			//X
			getline(is2,data_string, '\n'); 
			sensor.shape(j,0) = 0.01*std::stod(data_string); //convert cm to m
			//Arc legnth
			sensor.arc_length(j) = 0.001*j; //1 mm resolution, starting at 0
		}
		
		
		//Next entry is text field (skip) and again number of shape points (skip too)
		getline(is2,data_string, '\n');
		getline(is2,data_string, '\n'); 
		
		//Save all y values 
		for(int j = 0; j < sensor.num_shape_points; j++)
		{
			//X
			getline(is2,data_string, '\n'); 
			sensor.shape(j,1) = 0.01*std::stod(data_string); //convert cm to m
		}
		
		
		//Next entry is text field (skip) and again number of shape points (skip too)
		getline(is2,data_string, '\n');
		getline(is2,data_string, '\n'); 
		
		//Save all z values 
		for(int j = 0; j < sensor.num_shape_points; j++)
		{
			//Z
			getline(is2,data_string, '\n'); 
			sensor.shape(j,2) = 0.01*std::stod(data_string); //convert cm to m
		}
		
		
		//Next entry is either new curvature data (while loop will restart and add new sensor) or new line (no new sensor)
		getline(is2,data_string, '\n');
		
		sample.sensors.push_back(sensor);
		
		num_sensors++;
	}
	
	sample.num_sensors = num_sensors;
	
	std::cout << sample.sample_number << std::endl;
	std::cout << sample.num_channels << std::endl;
	std::cout << sample.num_sensors << std::endl << std::endl;
	
	for(int i = 0; i < sample.num_channels; i++)
	{
		std::cout << sample.channels.at(i).channel_number << std::endl;
		std::cout << sample.channels.at(i).num_gratings << std::endl << std::endl;
		
		std::cout << sample.channels.at(i).error_status(0) << std::endl;
		std::cout << sample.channels.at(i).error_status(1) << std::endl;
		std::cout << sample.channels.at(i).error_status(2) << std::endl;
		std::cout << sample.channels.at(i).error_status(3) << std::endl << std::endl;
		
		for(int j = 0; j < sample.channels.at(i).num_gratings; j++)
		{
			std::cout << sample.channels.at(i).peak_wavelengths(j) << " " << sample.channels.at(i).peak_powers(j) << std::endl;
		}
		std::cout << std::endl;
	}
	
	for(int i = 0; i < sample.num_sensors; i++)
	{
		std::cout << sample.sensors.at(i).num_curv_points << std::endl;
		std::cout << sample.sensors.at(i).num_shape_points << std::endl << std::endl;
		
		
		for(int j = 0; j < sample.sensors.at(i).num_curv_points; j++)
		{
			std::cout << sample.sensors.at(i).kappa(j) << " " << sample.sensors.at(i).phi(j) << std::endl;
		}
		std::cout << std::endl;
		
		
		for(int j = 0; j < sample.sensors.at(i).num_shape_points; j++)
		{
			std::cout << sample.sensors.at(i).shape.row(j) << " " << sample.sensors.at(i).arc_length(j) << std::endl;
		}
		std::cout << std::endl;
		
	}
	

	
	return true;
					
	
}

