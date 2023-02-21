/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

//includes
#include <iostream>

#include <Eigen/Dense>


#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "shape_sensing_interface.h"

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001"

int main(int argc, char **argv)
{
	
	try
	{
			
		//Set up variables
		boost::asio::io_context io_context;
		boost::asio::ip::tcp::resolver resolver(io_context);
		
		//Get endpoints
		boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(SERVER_ADDRESS, PORT_NUMBER);
		
		boost::asio::ip::tcp::endpoint ep = *endpoints;
		std::cout << ep << std::endl;
		
		std::cout << "Establishing connection to socket... "; 
		
		//Connect to socket and open connection
		boost::asio::ip::tcp::socket socket(io_context);
		boost::asio::connect(socket, endpoints);
		
		std::cout << "Connected!" << std::endl << std::endl;

		
		
		//Run in an endless loop and display received data
		while(1)
		{
			boost::array<char, 128> buf;
			boost::system::error_code error;

			size_t len = socket.read_some(boost::asio::buffer(buf), error);

			if (error == boost::asio::error::eof)
			{
				std::cout << "Connection closed by server." << std::endl << std::endl;
				break; // Connection closed cleanly by peer.
			}
			else if (error)
			{
				std::cout << "Error occured!" << std::endl << std::endl;
				throw boost::system::system_error(error); // Some other error.
			}

			std::cout << buf.data() << std::endl << std::endl;
		}
				
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}			
	return 0;
}

