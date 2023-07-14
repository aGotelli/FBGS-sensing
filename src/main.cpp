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

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001" //Default port number for Shape Sensing
//#define PORT_NUMBER "2055" //Default port number for Illumisense
//#define CALIB_FILE "/home/sven/Downloads/PR2022_14_53_01_S01 - calib.txt"

#include <algorithm>
#include <numeric>


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
    std::cout << "number of steps lost : " << step_jumps.size() << " On a total of " << steps.size() << " steps -> " << percentage << "%\n";
    if(not step_jumps.empty())
        std::cout << "Everage lost : " << total/step_jumps.size() << ", min lost : " << *min_max.first.base() << ", max lost : " << *min_max.second.base() << "\n\n";
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
//    interface.initialiseMemory(sample);
//    std::cout << "ok memory !" << std::endl;


    const double frequency = 200; //Hz
    const auto dt_ms = std::chrono::milliseconds(static_cast<unsigned int>((1.0/frequency)*1000));

    const unsigned int max_count = 1000;
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

                const auto row = sample.sensors[0].shape.rows() - 1;
                std::cout << "Tip pos : \n" << sample.sensors[0].shape.row(row) << "\n";

            }




        }

        std::this_thread::sleep_for(dt_ms );


    }


    processResult(sample_numbers);


//    std::cout << "\n\nNow new version \n\n" << std::endl;;

//    interface.m_data_stack.resize(max_count);

//    count = 0;
//    while(count<max_count)
//    {
//        if(interface.nextSampleReady())
//        {
//            if(interface.fetchDataFromTCPIP(count))
//            {
//                count++;
//            }

//        }

//        std::this_thread::sleep_for(dt_ms );

//    }

//    std::vector<unsigned int> sample_numbers_new(max_count);
//    for(unsigned int index=0; auto& sample_number : sample_numbers_new)
//        sample_number = interface.processDataAtIndex(index++).sample_number;






//    processResult(sample_numbers_new);




    return 1;



}
