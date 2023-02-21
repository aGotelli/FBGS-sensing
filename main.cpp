/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

//includes
#include <iostream>

#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "fbgs_interface.h"

//defines
#define SERVER_ADDRESS "127.0.0.1:5001"

int main(int argc, char **argv)
{
	
	try
	{
			
		//Set up variables
		boost::asio::io_context io_context;
		boost::asio::ip::tcp::resolver resolver(io_context);
		
		//Get endpoints
		boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(SERVER_ADDRESS, "");
		
		//Connect to socket and open connection
		boost::asio::ip::tcp::socket socket(io_context);
		boost::asio::connect(socket, endpoints);
		
		//Run in an endless loop and display received data
		while(1)
		{
			boost::array<char, 128> buf;
			boost::system::error_code error;

			size_t len = socket.read_some(boost::asio::buffer(buf), error);

			if (error == boost::asio::error::eof)
				break; // Connection closed cleanly by peer.
			else if (error)
				throw boost::system::system_error(error); // Some other error.

			std::cout.write(buf.data(), len);
		}
				
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}			
	return 0;
}

