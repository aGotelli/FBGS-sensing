/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#include "illumisense_interface.h"

IllumiSenseInterface::IllumiSenseInterface(std::string ip_address, std::string port_number) : 
	m_resolver(m_io_context), m_socket(m_io_context)
{
	m_ip_address = ip_address;
	m_port_number = port_number;
	m_connected = false;
}

IllumiSenseInterface::~IllumiSenseInterface()
{

}

bool IllumiSenseInterface::connect()
{
	m_connected = false;
	
	try
	{
		//Get endpoints
		boost::asio::ip::tcp::resolver::results_type endpoints = m_resolver.resolve(m_ip_address, m_port_number);
		
		boost::asio::ip::tcp::endpoint ep = *endpoints;
		
		std::cout << "Establishing connection to " << ep << " ... "; 
		
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

bool IllumiSenseInterface::nextSampleReady()
{
	if(m_connected)
	{
		return (m_socket.available() > 0);
	}
	else
	{
		std::cout << "TCP not connected!" << std::endl;
		return false;
	}
	
}

bool IllumiSenseInterface::readNextSample(Sample &sample)
{
	if(!m_connected || m_socket.available() <= 0)
	{
		
		std::cout << "TCP not connected or no data available!" << std::endl;
		return false;
	}
	else
	{
		
		try
		{
			boost::asio::streambuf data;
				
			//First read the first 4 bytes to figure out the size of the following ASCII string
			size_t size_read = boost::asio::read(m_socket,data,boost::asio::transfer_exactly(4));
			
			std::string data_string;
			std::istream is(&data);
			is >> data_string;
			
			std::string hex_string = stringToHex(data_string);
			
			int data_length = std::stoi(hex_string,0,16);
			
			
			//Now read the remaining ASCII string of the current data package
			size_read = boost::asio::read(m_socket,data,boost::asio::transfer_exactly(data_length));
			
			
			data_string.clear();
			std::istream is2(&data);
			
			//Create new, empty sample and set the passed sample to it
			Sample new_sample;
			sample = new_sample;
	
			//Skip first two entries (date and time)
			getline(is2,data_string, '\t'); 
			getline(is2,data_string, '\t'); 
			
			//Next string is the sample number
			getline(is2,data_string, '\t'); 
			sample.sample_number = std::stoi(data_string);
			
			//Next string is number of channels
			getline(is2,data_string, '\t'); 
			sample.num_channels = std::stoi(data_string);
			
			//Now we run through all channels
			for(int i = 0; i < sample.num_channels; i++)
			{
				IllumiSenseInterface::Sample::Channel channel;
				
				//Next string is channel number
				getline(is2,data_string, '\t'); 
				channel.channel_number = std::stoi(data_string);
				
				//Next string is number of gratings
				getline(is2,data_string, '\t'); 
				channel.num_gratings = std::stoi(data_string);
				
				//Next is error status
				getline(is2,data_string, '\t'); 
				channel.error_status(0) = std::stoi(data_string);
				getline(is2,data_string, '\t'); 
				channel.error_status(1) = std::stoi(data_string);
				getline(is2,data_string, '\t'); 
				channel.error_status(2) = std::stoi(data_string);
				getline(is2,data_string, '\t'); 
				channel.error_status(3) = std::stoi(data_string);
				
				//Next is peak wavelengths
				channel.peak_wavelengths.resize(channel.num_gratings);
				for(int j = 0; j < channel.num_gratings; j++)
				{
					getline(is2,data_string, '\t'); 
					channel.peak_wavelengths(j) = std::stod(data_string);
				}
				
				//Next is peak powers
				channel.peak_powers.resize(channel.num_gratings);
				for(int j = 0; j < channel.num_gratings; j++)
				{
					getline(is2,data_string, '\t'); 
					channel.peak_powers(j) = std::stod(data_string);
				}
				
				//Resize the vector of strains for later
				channel.strains.resize(channel.num_gratings);
				
				sample.channels.push_back(channel);
				
			}
			
			
			//Next is the number of engineered values (strain in our case)
			//We skip this value since this is the same as the number of all gratings over all channels
			getline(is2,data_string, '\t'); 
			
			
						
			for(int i = 0; i < sample.num_channels; i++)
			{
				for(int j = 0; j < sample.channels.at(i).num_gratings; j++)
				{
					getline(is2,data_string, '\t'); 
					sample.channels.at(i).strains = std::stod(data_string);
				}
			}
			
			//Create sensor data
						
			//Check the number of sensors
			//If we have four channels, each with the same number of gratings, we have one sensor
			//If we have eight channels, and the first four as well as the last four have the same number of gratings, we have two sensors
			//This is the maximum the hardware in the lab can currently support
			int num_sensors = 0;
			if(sample.num_channels == 4 && sample.channels(0).num_gratings == sample.channels(1).num_gratings && sample.channels(0).num_gratings == sample.channels(2).num_gratings && sample.channels(0).num_gratings == sample.channels(3).num_gratings)
			{
				num_sensors = 1
			}
			else if(sample.num_channels == 8 && sample.channels(0).num_gratings == sample.channels(1).num_gratings && sample.channels(0).num_gratings == sample.channels(2).num_gratings && sample.channels(0).num_gratings == sample.channels(3).num_gratings && sample.channels(4).num_gratings == sample.channels(5).num_gratings && sample.channels(4).num_gratings == sample.channels(6).num_gratings && sample.channels(4).num_gratings == sample.channels(7).num_gratings)
			{
				num_sensors = 2;
			}
			
			
			//For each sensor, computing the curvature strain of the multicore fiber at each grating position
			//Throughout this code, we are following the methods derived in:
			//Modes, Ortmaier, Burgner-Kahrs:
			//Shape Sensing Based on Longitudinal Strain Measurements Considering Elongation, Bending, and Twisting
			//IEEE Sensors Journal, 2020
			for(int i = 0; i < num_sensors; i++)
			{
				IllumiSenseInterface::Sample::Sensor sensor;
				
				sample.sensors.push_back(sensor);
			}
			
			return true;
					
		}
		catch(std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			return false;
		}
		
	}
	
	
}

std::string IllumiSenseInterface::stringToHex(const std::string& input)
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
