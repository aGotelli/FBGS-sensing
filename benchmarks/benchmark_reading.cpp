#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>


#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "fbgs-sensing/shape_sensing_interface.h"
#include "fbgs-sensing/illumisense_interface.h"

#include <benchmark/benchmark.h>

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001" //Default port number for Shape Sensing



const unsigned int repetitions = 5;


int main(int argc, char *argv[])
{
    ::benchmark::Initialize(&argc, argv);


    //Example code for Shape Sensing Interface

    ShapeSensingInterface interface(SERVER_ADDRESS,PORT_NUMBER);

    if(!interface.connect())
        return 0;


    ::benchmark::RegisterBenchmark("Reading data", [&](::benchmark::State &t_state){

        ShapeSensingInterface::Sample sample;

        int numb = 0;
        while(t_state.KeepRunning()){
            if(interface.nextSampleReady()){
                if(interface.readNextSample(sample)){
                    numb = sample.num_channels;
                }
            }
        }
    })->Repetitions(repetitions);



    ::benchmark::RunSpecifiedBenchmarks();


    return 0;
}
