// FILE			: manager_switch.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, January 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
//#define _SHOW_CALLBACK_

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <zeabus/ros/node.hpp>

#include    <zeabus_utility/MessagePlannerSwitch.h>

#include    <zeabus_utility/ServiceBool.h>

#include    <zeabus_utility/SendBool.h>

class ServiceResponseState
{
    public:
        ServiceResponseState( ros::NodeHandle* ptr_node_handle , 
                bool* ptr_state , 
                std::mutex* ptr_lock )
        {
            this->ptr_node_handle = ptr_node_handle;
            this->ptr_state = ptr_state;    
            this->ptr_lock = ptr_lock;
        }

        void setup_server()
        {
            this->service_reponse_state = this->ptr_node_handle->advertiseService( "/manage/switch",
                    &ServiceResponseState::callback_response_state , this );
        }

        bool callback_response_state( zeabus_utility::ServiceBool::Request& request ,
                zeabus_utility::ServiceBool::Response& response )
        {
            this->ptr_lock->lock();
            response.data = *(this->ptr_state);
            this->ptr_lock->unlock();
            return true;
        }

        ros::ServiceServer service_reponse_state;
        bool* ptr_state;
        ros::NodeHandle* ptr_node_handle;
        std::mutex* ptr_lock; 
};

int main( int argv , char** argc )
{

    zeabus_ros::Node node( argv , argc , "switch" );

    ros::NodeHandle ph("~");
    ros::NodeHandle nh("");

    node.spin();

    // start wilh false
    // state false mean switch turn off
    // start true mean switch turn on
    bool current_state = false;
    bool receive_message = true;
    unsigned int count_state = 0;
    const unsigned int limit_state = 20;

    // setup part subscriber planner switch
    std::mutex lock_planner_switch;
    zeabus_utility::MessagePlannerSwitch message_planner_switch;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::MessagePlannerSwitch > listener_switch( &nh,
            &message_planner_switch );
    listener_switch.setup_mutex_data( &lock_planner_switch );
    listener_switch.setup_subscriber_timestamp( "/planner_switch" , 1 );

    // setup service reponse current state 
    std::mutex lock_state;
    ServiceResponseState server_response_state( &nh , &current_state , &lock_state );
    server_response_state.setup_server();

    ros::Time time_stamp = ros::Time::now();

    // Part activity for send state 
    ros::ServiceClient client_activate_control = nh.serviceClient< zeabus_utility::SendBool >(
            "/control/activate" );
    ros::ServiceClient client_mission_project = nh.serviceClient< zeabus_utility::SendBool >(
            "/project/activate" );
    zeabus_utility::SendBool srv_bool;
    srv_bool.request.header.frame_id = "manager";

    // Part parameter to setup node
    
    int frequency;
    ph.param< int >( "frequency" , frequency , 10 );

    ros::Rate rate( frequency );

    while( ros::ok() )
    {
        rate.sleep();
        lock_planner_switch.lock();
        if( time_stamp == message_planner_switch.header.stamp )
        {
            if( receive_message )
            {
                std::cout   << "Warning ! message don't update current state : " 
                            << current_state << '\n';
                receive_message = false;
            }
        }
        else
        {
            time_stamp = message_planner_switch.header.stamp;
            if( message_planner_switch.planner_switch_state )
            {
                count_state++;
            }
            else
            {
                count_state--;
            }
            if( ! receive_message )
            {
                std::cout   << "Message receive\n";
                receive_message = true;
            }
        }
        lock_planner_switch.unlock();

decision_state:
        switch( count_state )
        {
        case -1 :
            count_state = 0;
            continue;
        case limit_state + 1:
            count_state = limit_state;
            continue;
        case 0 :
            if( current_state )
            {
                lock_state.lock();
                current_state = false;
                lock_state.unlock();
                goto activity_part;
            }
            continue;
        case limit_state:
            if( ! current_state )
            {
                lock_state.lock();
                current_state = true;
                lock_state.unlock();
                goto activity_part;
            }
            continue;
        }
        continue;

activity_part:
        if( current_state )
        {
            std::cout   << "Switch have been turn on\n";
            srv_bool.request.data = current_state;
        }
        else
        {
            std::cout   << "Switch have been turn off\n";
            srv_bool.request.data = current_state;
        }
        srv_bool.request.header.stamp = time_stamp;
        if( client_activate_control.call( srv_bool ) )
        {
            std::cout   << "\tCONTROL : Success\n";
        }
        else
        {
            std::cout   << "\tCONTROL : Failure\n";
        }
        
        if( client_mission_project.call( srv_bool ) )
        {
            std::cout   << "\tMISSION : Success\n";
        }
        else
        {
            std::cout   << "\tMISSION : Failure\n";
        }
    } // main loop

exit_main:
    ros::shutdown(); 
}
