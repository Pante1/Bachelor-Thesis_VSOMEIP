// Copyright (C) 2014-2023 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//This file is part of the VSOMEIP example code, originally retrieved from:
//https://github.com/COVESA/vsomeip/tree/master/examples
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
#include <csignal>
#endif
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <vsomeip/vsomeip.hpp>

#include "sample-ids.hpp"

//C Code Utilities and Setup **************************************************************/
#include <iostream>
#include <fcntl.h>       // For open(), O_RDONLY, O_CREAT, etc.
#include <sys/ipc.h>     // For IPC_CREAT, shmget(), shmat(), shmdt()
#include <sys/shm.h>     // For shared memory functions
#include <unistd.h>      // For close(), unlink()
#include <cstring>       // For perror(), strerror()
#include <ctime>         // For clock_gettime()
#include <sys/stat.h>    // Für mkfifo()
//*******************************************************************/

const char* PathToObjectDetectionModel = "/home/raspberry/Desktop/YOLO/Ultralytics_YOLOv8/LogoRecognitionFromCamera_With_SharedMemory2.py";
//const char* PathToObjectDetectionModel = "ObjectDetectionModel.py";

//Time **************************************************************/
#define NUM_MEASUREMENTS 20
static long gStartTimestamp_Sec;
static long gStartTimestamp_Nsec;
static long gEndTimestamp_Sec;
static long gEndTimestamp_Nsec;
static int timeMeasurementIndex = 0;

long getTimestamp_Sec();
long getTimestamp_Nsec();
//*******************************************************************/

//Shared Memory *****************************************************/
#define SHMKEY 4711
#define PIPMODE 0600

#define BUFFER_SIZE 1024
char gSendDataBuffer[BUFFER_SIZE];

pid_t childProcessId = -1;

typedef struct {
	bool logosDetected;
    int logos[4];
    long timestampMemoryInsertion_Sec;
    long timestampMemoryInsertion_Nsec;
    } gLogoDetectionDataset;

gLogoDetectionDataset *gLogoDetection = nullptr;

int gFd_FIFO = -1;
int gShmId = -1;
char gMsgBuffer_FIFO[16];
int gReadBytes_FIFO = -1;//currently not used

int initializeFIFOAndSharedMemory();
void detachSharedMemoryAndClosePipe();
//*******************************************************************/

long getTimestamp_Sec()
{
	struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec;
}

long getTimestamp_Nsec(void)
{
	struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_nsec;
}

int initializeFIFOAndSharedMemory() {
    int errorFlag = -1;

    //If the program is terminated with Ctrl+C
    shmdt(gLogoDetection);
    close(gFd_FIFO);
    unlink("/home/raspberry/Desktop/GitHub_UDP/FIFO");

    if (mkfifo("/home/raspberry/Desktop/GitHub_UDP/FIFO", PIPMODE) < 0) {
        perror("Error creating FIFO");
    } else {
        gFd_FIFO = open("/home/raspberry/Desktop/GitHub_UDP/FIFO", O_RDONLY, 0);
        if (gFd_FIFO < 0) {
            perror("Error opening FIFO for reading");
        } else {
            gShmId = shmget(SHMKEY, sizeof(gLogoDetectionDataset), IPC_CREAT | PIPMODE);
            if (gShmId < 0) {
                perror("Error requesting shared memory");
            } else {
                gLogoDetection = (gLogoDetectionDataset*)shmat(gShmId, nullptr, 0);
                if (gLogoDetection == (gLogoDetectionDataset*)-1) {
                    perror("Error attaching shared memory");
                } else {
                    errorFlag = 0;
                }
            }
        }
    }

    return errorFlag;
}

void detachSharedMemoryAndClosePipe() {
    if (shmdt(gLogoDetection) == -1){
        perror("Error detaching shared memory");
    }

    if (close(gFd_FIFO) == -1) {
        perror("Error closing FIFO");
    }

    if (unlink("/home/raspberry/Desktop/GitHub_UDP/FIFO") == -1) {
        perror("Error deleting FIFO");
    }
}
//////////////////////////////////////////////////////////////////////////////////

class service_sample {
public:
    service_sample(bool _use_static_routing) :
            app_(vsomeip::runtime::get()->create_application()),
            is_registered_(false),
            use_static_routing_(_use_static_routing),
            blocked_(false),
            running_(true),
            offer_thread_(std::bind(&service_sample::run, this)) {
    }

    bool init() {
        std::lock_guard<std::mutex> its_lock(mutex_);

        if (!app_->init()) {
            std::cerr << "Couldn't initialize application" << std::endl;
            return false;
        }
        app_->register_state_handler(
                std::bind(&service_sample::on_state, this,
                        std::placeholders::_1));
        app_->register_message_handler(
                SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_METHOD_ID,
                std::bind(&service_sample::on_message, this,
                        std::placeholders::_1));

        std::cout << "Static routing " << (use_static_routing_ ? "ON" : "OFF")
                  << std::endl;
        return true;
    }

    void start() {
        app_->start();
    }

    void stop() {
        running_ = false;
        blocked_ = true;
        app_->clear_all_handler();
        stop_offer();
        condition_.notify_one();
        if (std::this_thread::get_id() != offer_thread_.get_id()) {
            if (offer_thread_.joinable()) {
                offer_thread_.join();
            }
        } else {
            offer_thread_.detach();
        }

        detachSharedMemoryAndClosePipe();//-----------------------------
		if (kill(childProcessId, SIGKILL) == -1)
		{
            perror("Error killing the process");
		}//-------------------------------------------------------------

        app_->stop();
    }

    void offer() {
        app_->offer_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
        app_->offer_service(SAMPLE_SERVICE_ID + 1, SAMPLE_INSTANCE_ID);
    }

    void stop_offer() {
        app_->stop_offer_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
        app_->stop_offer_service(SAMPLE_SERVICE_ID + 1, SAMPLE_INSTANCE_ID);
    }

    void on_state(vsomeip::state_type_e _state) {
        std::cout << "Application " << app_->get_name() << " is "
                << (_state == vsomeip::state_type_e::ST_REGISTERED ?
                        "registered." : "deregistered.")
                << std::endl;

        if (_state == vsomeip::state_type_e::ST_REGISTERED) {
            if (!is_registered_) {
                is_registered_ = true;
                blocked_ = true;
                condition_.notify_one();
            }
        } else {
            is_registered_ = false;
        }
    }

    void on_message(const std::shared_ptr<vsomeip::message> &_request) {
        std::shared_ptr<vsomeip::message> its_response = vsomeip::runtime::get()->create_response(_request);
        std::shared_ptr<vsomeip::payload> its_payload = vsomeip::runtime::get()->create_payload();

        std::vector<vsomeip::byte_t> its_payload_data;
  
        for(int i = 0; i < 4; i++){
            its_payload_data.push_back(static_cast<vsomeip::byte_t>(1));//the vector is filled with pseudo number
        }
            
        its_payload->set_data(its_payload_data);
        its_response->set_payload(its_payload);
        app_->send(its_response);
    }

    void run() {
        std::unique_lock<std::mutex> its_lock(mutex_);
        while (!blocked_)
            condition_.wait(its_lock);

        offer();
        std::cout << "Service is now being offered continuously." << std::endl;

        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

private:
    std::shared_ptr<vsomeip::application> app_;
    bool is_registered_;
    bool use_static_routing_;

    std::mutex mutex_;
    std::condition_variable condition_;
    bool blocked_;
    bool running_;

    // blocked_ must be initialized before the thread is started.
    std::thread offer_thread_;
};

#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
    service_sample *its_sample_ptr(nullptr);
    void handle_signal(int _signal) {
        if (its_sample_ptr != nullptr && (_signal == SIGINT || _signal == SIGTERM)){
            its_sample_ptr->stop();

            detachSharedMemoryAndClosePipe();//-----------------------------
		    if (kill(childProcessId, SIGKILL) == -1) {
                perror("Error killing the process");
		    }//-------------------------------------------------------------
        }
    }
#endif

int main(int argc, char **argv) {

    childProcessId = fork();
    if (childProcessId < 0) {
        std::cerr << "Error with fork: " << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }
    else if (childProcessId == 0) {
        if (execlp("python3", "python3", PathToObjectDetectionModel, (char*)nullptr) == -1) {
            std::cerr << "Error executing Python script: " << strerror(errno) << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    else {
        if (initializeFIFOAndSharedMemory() != 0) {
            printf("Error with initializeFIFOAndSharedMemory().\n");
            exit(EXIT_FAILURE);
        }

        bool use_static_routing(false);

        std::string static_routing_enable("--static-routing");

        for (int i = 1; i < argc; i++) {
            if (static_routing_enable == argv[i]) {
                use_static_routing = true;
            }
        }

        service_sample its_sample(use_static_routing);
    #ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
        its_sample_ptr = &its_sample;
        signal(SIGINT, handle_signal);
        signal(SIGTERM, handle_signal);
    #endif
        if (its_sample.init()) {
            its_sample.start();
    #ifdef VSOMEIP_ENABLE_SIGNAL_HANDLING
            its_sample.stop();
    #endif
            return 0;
        } else {
            return 1;
        }
    }
}