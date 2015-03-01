// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <dirent.h>
#include <map>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
using namespace std;

int directory_mov = 0;
unsigned long directory_size = 0;
int commands_mov = 0;

int rest_mov = 0;
int wavein_mov = 0;
int waveout_mov = 0;
int fingersspread_mov = 0;
int doubletap_mov = 0;
int fist_mov = 0;
bool commands_list = false;

vector<string> commands;
string commit = "myo commit";
int commit_num = 1;
unsigned long commands_size;

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener
{
public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }

    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }

    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        if (pose != myo::Pose::unknown && pose != myo::Pose::rest)
        {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);

            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
        }
        else
        {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            //myo->unlock(myo::Myo::unlockTimed);
            myo->unlock(myo::Myo::unlockHold);
        }
    }

    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        onArm = true;
        whichArm = arm;
    }

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }

    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }

    /*function... might want it in some class?*/
    int getdir (string dir, vector<string> &files)
    {
        DIR *dp;
        struct dirent *dirp;
        if((dp  = opendir(dir.c_str())) == NULL)
        {
            cout << "Error(" << errno << ") opening " << dir << endl;
            return errno;
        }
        
        while ((dirp = readdir(dp)) != NULL)
        {
            files.push_back(string(dirp->d_name));
        }
        closedir(dp);
        return 0;
    }
    
    
    vector<string> updateDir()
    {
        string dir = string(".");
        vector<string> files = vector<string>();
        getdir(dir,files);
        return files;
    }
    
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        if (onArm)
        {
//            cout << "test";
            std::string poseString = currentPose.toString();
            if (poseString == "rest")
            {
                rest_mov +=1;
                wavein_mov = 0;
                waveout_mov = 0;
                fingersspread_mov = 0;
                rest_mov %= 21;
                fist_mov = 0;
                if(rest_mov == 20)
                {
                }
            }
            else if (poseString == "waveIn")
            {
                wavein_mov += 1;
                waveout_mov = 0;
                fingersspread_mov = 0;
                fist_mov = 0;
                rest_mov = 0;
                wavein_mov %= 21;
                if(wavein_mov == 20)
                {
                    if(commands_list == false)
                    {
                        directory_mov -= 1;
                        vector<string> files = updateDir();
                        directory_mov = (directory_mov + files.size()) % files.size();
                        for (unsigned int i = 0;i < files.size();i++)
                        {
                            if(i == directory_mov)
                                cout << "* " << files[i] << endl;
                            else
                                cout << files[i] << endl;
                        }
                    }
                    else
                    {
////                        commands_mov--;
//                        commands_mov = (commands_mov + commands.size() - 1) % commands.size();
                        
                        for(unsigned int i = 0; i < commands_size; i++)
                        {
                            if(i == commands_mov)
                                cout << "* " << commands[i] << endl;
                            else
                                cout << commands[i] << endl;
                        
                        }
                        
                    }
                }
            
            }
            else if (poseString == "waveOut")
            {
                waveout_mov +=1;
                waveout_mov %= 21;
                fingersspread_mov = 0;
                fist_mov = 0;
                rest_mov = 0;
                wavein_mov = 0;
//                cout << "wave out detected";
                if(waveout_mov == 20)
                {
//                    cout<<"test";
                    if(commands_list == false)
                    {
                        directory_mov += 1;
                        vector<string> files = updateDir();
                        directory_mov = (directory_mov + files.size()) % files.size();

                        for (unsigned int i = 0;i < files.size();i++)
                        {
                            if(i == directory_mov)
                                cout << "* " << files[i] << endl;
                            else
                                cout << files[i] << endl;
                        }
                    }
                    else
                    {
                        commands_mov += 1;
                        commands_mov = (commands_mov + commands.size()) % commands.size();
                        
                        for(unsigned int i= 0; i < commands.size(); i++)
                        {
                            if(i == commands_mov)
                                cout << "* " << commands[i] << endl;
                            else
                                cout << commands[i] << endl;
                            
                        }
                    }
                }
            }
            else if (poseString == "fingersSpread")
            {
                fingersspread_mov +=1;
                waveout_mov = 0;
                wavein_mov = 0;
                fingersspread_mov %= 21;
                fist_mov = 0;
                rest_mov = 0;
                if (fingersspread_mov == 20)
                {
                    if(commands_list == true)
                    {
                        if(commands_mov != 4){
                            vector<string> files = updateDir();
                            system((commands[commands_mov] + files[directory_mov]).c_str());
                        }
                        
                       commands_list=false;
                    }
                    else
                    {
                        commands_list = true;
                        commands_mov = 0;
//                        cout << "stuck" << endl;
                        for(unsigned int i = 0; i < commands.size(); i++)
                        {
                            if( i == commands_mov)
                                cout << "* " << commands[i] << endl;
                            else
                                cout << commands[i] << endl;
                        }
                        
//                        cout << "GOT OUT"<< endl;
                    }
                }
            }
            
            else if (poseString == "fist")
            {
                fist_mov +=1;
                fingersspread_mov = 0;
                rest_mov = 0;
                waveout_mov = 0;
                wavein_mov = 0;
                fist_mov %= 21;
//                std::cout<< "fist";
                
                if (fist_mov == 20)
                {
                    chdir("..");
                    directory_mov = 0;
                    vector<string> files = updateDir();
                    for (unsigned int i = 0;i < files.size();i++)
                    {
                        if(i == directory_mov)
                            cout << "* " << files[i] << endl;
                        else
                            cout << files[i] << endl;
                    }
                }
            }
            else
            {
            }
        }
        else
        {
        }
        std::cout << std::flush;
    }

    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;

    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;

    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
    
};



int main(int argc, char** argv)
{

    commands.push_back("cat ");
    commands.push_back("git add ");
    commands.push_back("git commit ");
    commands.push_back("git push ");
    commands.push_back("exit commands list ");
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try
    {
        myo::Hub hub("com.example.hello-myo");

        std::cout << "Attempting to find a Myo..." << std::endl;

        myo::Myo* myo = hub.waitForMyo(10000);
        if (!myo)
        {
            throw std::runtime_error("Unable to find a Myo!");
        }
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
        DataCollector collector;

        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);

        // Finally we enter our main loop.
        while (1)
        {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            hub.run(1000/20);
            // After processing events, we call the print() member function we defined above to print out the values we've
            // obtained from any events that have occurred.
            collector.print();
        }

    // If a standard exception occurred, we print out its message and exit.
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
