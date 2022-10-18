#include <std_msgs/Bool.h>

#include <awesome_leg/JumpNow.h>
#include <awesome_leg/MatReplayerStatus.h>

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include <time.h>

#include <chrono>
#include <thread>

class CyclicJumpTrig
{
    public:

        CyclicJumpTrig(ros::NodeHandle *nh, double replay_pause_time = 1.0)
        :_nh{nh}, _replay_pause_time{replay_pause_time}
        {
            _approach_traj_finished = false;
            _traj_finished = false;
            
            _jump_state_topicname = "";

        }

        void start_client(std::string full_topicname = "/mat_replayer_rt/my_jump_now")
        {
            _jump_state_topicname = full_topicname;

            _client = _nh->serviceClient<awesome_leg::JumpNow>(_jump_state_topicname);

            _srv.request.jump_now = true; // initializing the node triggers the jump sequence
            
        }

        void start_subscriber()
        {
            _sub = _nh->subscribe("/mat_replayer_rt/replay_status_node", 1000, &CyclicJumpTrig::replay_status_callback, this);
        }

        void spin_node()
        {
            
            ros::spin(); // all callbacks will be called 
            //within the spin method, which is blocking
            
        }

    private:

        bool _approach_traj_finished;
        bool _traj_finished;
        bool _traj_sign_already_sent = false;
        bool _approach_sign_already_sent = false;

        bool _pause_started = false;

        double _replay_pause_time; 
        double _pause_timer = 0.0;
        
        struct timer
        {
            typedef std::chrono::steady_clock clock ;
            typedef std::chrono::seconds seconds ;

            void reset() { start = clock::now() ; }

            unsigned long long seconds_elapsed() const
            { return std::chrono::duration_cast<seconds>( clock::now() - start ).count() ; }

            private: clock::time_point start = clock::now() ;
        };

        timer _timer;

        std::string _jump_state_topicname;

        ros::NodeHandle *_nh;
        ros::ServiceClient _client;

        awesome_leg::JumpNow _srv;

        ros::Subscriber _sub;

        void replay_status_callback(const awesome_leg::MatReplayerStatus& msg)
        {
            _approach_traj_finished = msg.approach_traj_finished;

            _traj_finished = msg.traj_finished;

            print_traj_status();
            
            if(!_approach_traj_finished && !_traj_finished && !_pause_started && !_approach_sign_already_sent)
            { // sending approach trajectory start signal
                if (_client.call(_srv))
                {
                    ROS_INFO("\nSending signal for starting approach traj. ...\n");
                }
                else
                {
                    ROS_ERROR("Failed to call service cyclic_jump_triggerer");
                }

                _pause_started = false;

                _approach_sign_already_sent = true;
            }

            if(_approach_traj_finished  && !_traj_finished && !_pause_started && !_traj_sign_already_sent)
            { // sending trajectory start signal
                if (_client.call(_srv))
                {
                    ROS_INFO("\nSending signal for starting traj. ...\n");
                }
                else
                {
                    ROS_ERROR("Failed to call service cyclic_jump_triggerer");
                }

                _pause_started = false;

                _traj_sign_already_sent = true;
            }

            if(_approach_traj_finished  && _traj_finished && !_pause_started)
            {
                _timer.reset() ; 
                _pause_started = true;
            }

            if(_pause_started)
            {
                double elapsed_seconds = _timer.seconds_elapsed();

                if( elapsed_seconds > _replay_pause_time)
                { 
                    _timer.reset() ; 
                    _pause_started = false; // trigger next jump sequence
                    _approach_sign_already_sent = false;
                    _traj_sign_already_sent = false;
                    
                    ROS_INFO("\npause ended\n");

                }
                else
                {
                    ROS_INFO("\npause_timer: %f\n", elapsed_seconds);
                }
            }

        }

        void print_traj_status()
        {
            std::cout << "" << std::endl;

            ROS_INFO("\n Approach_traj_finished: %i\n", (int)_approach_traj_finished);
            ROS_INFO("\n traj_finished: %i\n", (int)_traj_finished);
            ROS_INFO("\n _pause_started: %i\n", (int)_pause_started);

            std::cout << "" << std::endl;
        }

};


int main(int argc, char **argv)
{   
    std::string node_name = "cyclic_jump_triggerer_client";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh; 
    
    double pause_time = (argc == 1) ? 3.0 : atof(argv[1]); 
    
    CyclicJumpTrig cyclic_jump_triggerer = CyclicJumpTrig(&nh, pause_time);
    
    cyclic_jump_triggerer.start_client();

    cyclic_jump_triggerer.start_subscriber();

    cyclic_jump_triggerer.spin_node();

    return 0;
}

