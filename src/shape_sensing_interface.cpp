/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#include "shape_sensing_interface.h"

ShapeSensingInterface::ShapeSensingInterface(std::string ip_address, std::string port_number) : 
	m_resolver(m_io_context), m_socket(m_io_context)
{
	m_ip_address = ip_address;
	m_port_number = port_number;
	m_connected = false;

}

ShapeSensingInterface::~ShapeSensingInterface()
{

}

bool ShapeSensingInterface::connect()
{
	m_connected = false;
	
	try
	{
		//Get endpoints
		std::cout << "[FBGS] Get endpoints... ";
		boost::asio::ip::tcp::resolver::results_type endpoints = m_resolver.resolve(m_ip_address, m_port_number);
		
		boost::asio::ip::tcp::endpoint ep = *endpoints;
		
		std::cout << "Establishing connection to " << ep << "... "; 
		
		//Connect to socket and open connection
		boost::asio::connect(m_socket, endpoints);
		
		std::cout << "Connected!" << std::endl << std::endl;
		
		m_connected = true;
		
		return true;	
				
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return false;
	}
	
	
}

bool ShapeSensingInterface::nextSampleReady()
{
	if(m_connected)
	{
		return (m_socket.available() >= 4);
	}
	else
	{
		std::cout << "[FBGS] TCP not connected!" << std::endl;
		return false;
	}
	
}

bool ShapeSensingInterface::readNextSample(Sample &sample)
{
	if(!m_connected)
	{
		
		std::cout << "[FBGS] TCP not connected!" << std::endl;
		return false;
	}
	else if(m_socket.available() < 4)
	{
		std::cout << "[FBGS] Data not ready!" << std::endl;
		return false;
	}	
	else
	{
		
		try
		{
			char buffer[4];
			//First read the first 4 bytes to figure out the size of the following ASCII string
			size_t size_read = boost::asio::read(m_socket,boost::asio::buffer(&buffer,4));
			
			//Convert the received bytes to signed integer
			int size = int((unsigned char)(buffer[0]) << 24 |
            (unsigned char)(buffer[1]) << 16 |
            (unsigned char)(buffer[2]) << 8 |
            (unsigned char)(buffer[3]));
			
			
			boost::asio::streambuf data;
			//Now read the remaining ASCII string of the current data package
			size_read = boost::asio::read(m_socket,data,boost::asio::transfer_exactly(size));
			
			
			std::string data_string;
			std::istream is(&data);
			
			//Create new, empty sample and set the passed sample to it
			Sample new_sample;
			sample = new_sample;
			
			//Skip first two entries (date and time)
			getline(is,data_string, '\t'); 
			getline(is,data_string, '\t'); 
			
			//Next string is the sample number
			getline(is,data_string, '\t'); 
			sample.sample_number = std::stoi(data_string);
			
			//Next string is number of channels
			getline(is,data_string, '\t'); 
			sample.num_channels = std::stoi(data_string);
			
			//Now we run through all channels
			for(int i = 0; i < sample.num_channels; i++)
			{
				ShapeSensingInterface::Sample::Channel channel;
				
				//Next string is channel number
				getline(is,data_string, '\t'); 
				channel.channel_number = std::stoi(data_string);
				
				//Next string is number of gratings
				getline(is,data_string, '\t'); 
				channel.num_gratings = std::stoi(data_string);
				
				//Next is error status
				getline(is,data_string, '\t'); 
				channel.error_status(0) = std::stoi(data_string);
				getline(is,data_string, '\t'); 
				channel.error_status(1) = std::stoi(data_string);
				getline(is,data_string, '\t'); 
				channel.error_status(2) = std::stoi(data_string);
				getline(is,data_string, '\t'); 
				channel.error_status(3) = std::stoi(data_string);
				
				//Next is peak wavelengths
				channel.peak_wavelengths.resize(channel.num_gratings);
				for(int j = 0; j < channel.num_gratings; j++)
				{
					getline(is,data_string, '\t'); 
					channel.peak_wavelengths(j) = std::stod(data_string);
				}
				
				//Next is peak powers
				channel.peak_powers.resize(channel.num_gratings);
				for(int j = 0; j < channel.num_gratings; j++)
				{
					getline(is,data_string, '\t'); 
					channel.peak_powers(j) = std::stod(data_string);
				}	
				
				sample.channels.push_back(channel);
				
			}
			
			
			//Now run through the file to the end
			getline(is,data_string, '\t'); 
			int num_sensors = 0;
			while(data_string == "Curvature [1/cm]")
			{
				ShapeSensingInterface::Sample::Sensor sensor;
				sensor.num_curv_points = sample.channels.at(0 + 4*num_sensors).num_gratings;
				
				//Save kappa (curvature) values
				sensor.kappa.resize(sensor.num_curv_points);
				for(int j = 0; j < sensor.num_curv_points; j++)
				{
					getline(is,data_string, '\t'); 
					sensor.kappa(j) = 100*std::stod(data_string); //convert 1/cm to 1/m	
				}	
				
				//Next entry is text field (skip)
				getline(is,data_string, '\t'); 
				
				//Save phi (curvature angle) values in rad
				sensor.phi.resize(sensor.num_curv_points);
				for(int j = 0; j < sensor.num_curv_points; j++)
				{
					getline(is,data_string, '\t'); 
					sensor.phi(j) = std::stod(data_string);
				}
				
				//Next entry is text field (skip)
				getline(is,data_string, '\t'); 	
				
				//Next entry is number of shape points
				getline(is,data_string, '\t'); 	
				sensor.num_shape_points = std::stoi(data_string);
				
				sensor.shape.resize(sensor.num_shape_points,3);
				sensor.arc_length.resize(sensor.num_shape_points);
				
				//Save all x values and arclength values
				for(int j = 0; j < sensor.num_shape_points; j++)
				{
					//X
					getline(is,data_string, '\t'); 
					sensor.shape(j,0) = 0.01*std::stod(data_string); //convert cm to m
					//Arc legnth
					sensor.arc_length(j) = 0.001*j; //1 mm resolution, starting at 0
				}
				
				
				//Next entry is text field (skip) and again number of shape points (skip too)
				getline(is,data_string, '\t');
				getline(is,data_string, '\t'); 
				
				//Save all y values 
				for(int j = 0; j < sensor.num_shape_points; j++)
				{
					//X
					getline(is,data_string, '\t'); 
					sensor.shape(j,1) = 0.01*std::stod(data_string); //convert cm to m
				}
				
				
				//Next entry is text field (skip) and again number of shape points (skip too)
				getline(is,data_string, '\t');
				getline(is,data_string, '\t'); 
				
				//Save all z values 
				for(int j = 0; j < sensor.num_shape_points; j++)
				{
					//Z
					getline(is,data_string, '\t'); 
					sensor.shape(j,2) = 0.01*std::stod(data_string); //convert cm to m
				}
				
				
				//Next entry is either new curvature data (while loop will restart and add new sensor) or new line (no new sensor)
				getline(is,data_string, '\t');
				
				sample.sensors.push_back(sensor);
				
				num_sensors++;
			}
			
			sample.num_sensors = num_sensors;
			
			return true;
					
		}
		catch(std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			return false;
		}
		
	}
	
	
}
