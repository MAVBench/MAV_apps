#include "ros/ros.h"
#include "ros/this_node.h" 
#include "ros/master.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
//#include "debugstream.h"
#include "options.h"
#include "regManip.h"
#include "utils.h"
#include "addrSpace.h"
//#include "statistics.h"
#include <std_srvs/Empty.h>
#include <ros/duration.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/user.h>
#include <sys/reg.h>
#include <errno.h>
#include <thread>
#include <inttypes.h>
#include <bitset>
#include <time.h>
#include <iostream>
#include <vector>
#include <string>
#include <cctype>
#include <iomanip>  // std::setprecision()
#include <fstream>
#include <mavbench_msgs/rosfi.h>
/*#include "../regManip.h"
#include "../utils.h"
#include "../debugstream.h"
#include "../statistics.h"
#include "../options.h"
*/


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
bool FI_start = false;
int traced_process_ready = 0;
std::vector<int> traced_process_id;
std::vector<std::string> traced_process_name;
int num_flips;
int range_select_ros = 2;
int num_traced_process = 3;
bool waiting = true;
bool succeed = false;
int process_select;
std::string modified_reg;
std::list<double> inject_time;
void PIDCallback(const mavbench_msgs::rosfi::ConstPtr& msg)
{
  traced_process_id.push_back(msg->pid);
  traced_process_name.push_back(msg->name);
  ROS_INFO("traced_process %s id: %d", traced_process_name[traced_process_name.size() - 1].c_str(), traced_process_id[traced_process_id.size() - 1]);
  if(traced_process_id.size() == num_traced_process){
    FI_start = true;
  }
}
void traced_processCallback(const std_msgs::Int32::ConstPtr& msg)
{
  traced_process_ready = msg->data;
}

bool doBitFlip(int num_flips) {
  // wait for the process to stop
  usleep(150);
  int target_process;
  if(process_select == 0){
    for (int i = 0; i < traced_process_id.size(); ++i){
      if(traced_process_name[i] == "occupancy_map_node"){
        target_process = i;
      }
    }
  }
  else if(process_select == 1){
    for (int i = 0; i < traced_process_id.size(); ++i){
      if(traced_process_name[i] == "follow_trajectory"){
        target_process = i;
      }
    }
  }
  int injected_process_id = traced_process_id[target_process];
  ROS_INFO("start fault injection to process %s: %d", traced_process_name[target_process].c_str(), injected_process_id);
  //assert(traced_process_id > 0 && "Uninitialized?");
  bool check_reg = false;
  RegisterManipulator RM(injected_process_id);
  RegDescr Reg;
  unsigned StartBit;
  bool Success;
  while(!check_reg){
    for (int i = 0; i < traced_process_id.size(); ++i){
      if(ptrace(PTRACE_INTERRUPT, traced_process_id[i], 0, 0) == -1){
          succeed = false;
          ROS_INFO("ptrace interrupt failed: %d\n", errno);
          if (errno == EBUSY){
            ROS_INFO("EBUSY");
          }
          else if (errno == EFAULT){
            ROS_INFO("EFAULT");
          }
          else if (errno == EINVAL){
            ROS_INFO("EINVAL");
          }
          else if(errno == EPERM){
            ROS_INFO("EPERM");
          }
          else if(errno == EIO){
            ROS_INFO("EIO");
          }
          else if(errno == ESRCH){
            ROS_INFO("ESRCH");
          }
      }
    }
    usleep(200);
    ROS_INFO("get program counter");
    uint8_t *IP = RM.getProgramCounter();
    //ROS_INFO("IP: %d", IP);
    // Parse the address space of the child.
    ROS_INFO("parse target process address space");
    AddressSpace ChildAS(injected_process_id);

    // Check if we are in library
    if (DontInjectToLibs && ChildAS.isInLibrary((uint64_t)IP)) {
      //dbg(2) << "We are in a library, skipping\n";
      ROS_INFO("We are in a library, skipping\n");
    }

    ROS_INFO("try to get random reg and bit");
    check_reg = true;
    std::tie(Reg, StartBit, Success) = RM.getRandomRegAndBit(IP);
    printf("Reg.Name: %s \n", Reg.Name.c_str());
    modified_reg = Reg.Name.c_str();
    if(Reg.Name.compare("bl")==0 || Reg.Name.compare("cl")==0 || Reg.Name.compare("dl")==0 || Reg.Name.compare("rip")==0 || Reg.Name.compare("r14")==0 || Reg.Name.compare("r15")==0 || Reg.Name.compare("rdi")==0 || Reg.Name.compare("rsi")==0 || Reg.Name.compare("rax")==0 || Reg.Name.compare("rbx")==0 || Reg.Name.compare("rcx")==0 || Reg.Name.compare("rdx")==0 || Reg.Name.compare("rbp")==0 || Reg.Name.compare("rsp")==0 || Reg.Name.compare("esi")==0 || Reg.Name.compare("edi")==0 || Reg.Name.compare("eax")==0 || Reg.Name.compare("ecx")==0 || Reg.Name.compare("edx")==0 || !Success){
    //if( Reg.Name.compare("r14")==0 || Reg.Name.compare("r15")==0 || Reg.Name.compare("rsp")==0 || Reg.Name.compare("rcx")==0 || Reg.Name.compare("rbx")==0 || Reg.Name.compare("rdx")==0 || Reg.Name.compare("esi")==0 || Reg.Name.compare("rdi")==0 || Reg.Name.compare("rsi")==0 || !Success){
      check_reg = false;
      for (int i = 0; i < traced_process_id.size(); ++i){
        if(ptraceSafe(PTRACE_CONT, traced_process_id[i], 0, 0) == -1){
          succeed = false;
        }
      }
      //ptraceSafe(PTRACE_SINGLESTEP, injected_process_id, 0, 0);
      printf("re-fech instruction\n");
      usleep(50);
    }
  }
  // If we are injecting the fault into a register that gets written, then step
  // to the next instruction before inject it, otherwise the bitflip will be
  // overwritten by the output of the current instruction.
  
  //dbg(3) << "Reg to be modified: " << Reg.dumpStr() << "\n";
  if (Reg.Written) {
    // Step to next instruction.
    ROS_INFO("about to SINGLESTEP\n");
    //dbg(2) << "about to SINGLESTEP\n";
    if(ptraceSafe(PTRACE_SINGLESTEP, injected_process_id, 0, 0) == -1){
      succeed = false;
    }
    usleep(200);
    /*int Status;
    const auto &StatusPid = waitpidSafe(traced_process_id);
    ExitState State = getWaitPidExitState(StatusPid.Status);
    // If we are *exteremely* unlucky, the alarm might have gone off.
    // We will just restart test run.
    if (State.Type == ExitType::Stopped && State.Val == SIGALRM)
      return false;
    assert(State.Type == ExitType::Stopped && State.Val == SIGTRAP &&
           "SingleStep should stop the program with a SIGTRAP.");*/
  }
  unsigned Bits = Reg.Bits;
  std::vector<int> record;
  bool success_select = false;
  int select;
  ROS_INFO("Reg to be modified: %s bitwidth: %d", Reg.Name.c_str(), Bits);
  for(int i = 0; i < num_flips; i++){
    while(!success_select){
      if(Bits == 64){
        if(range_select_ros == 1){
            select = 63;
        }
        else if(range_select_ros == 2){
            select = rand() % 11 + 52;
        }
        else if(range_select_ros == 3){
            select = rand() % 52;
        }
      }
      else if(Bits == 128){
        if(range_select_ros == 1){
            select = 127;
        }
        else if(range_select_ros == 2){
            select = rand() % 17 + 110;
        }
        else if(range_select_ros == 3){
            select = rand() % 110;
        }
      }
      else{
        select = randSafe(Bits);
      }
      success_select = true;
      if(!record.empty()){
          for (int j = 0; j < record.size(); ++j){
              if(record[j] == (select)){
                  success_select = false;
              }
          }
      }
    }
    // Now try to flip the bit.
    if (!RM.tryBitFlip(Reg.Name, StartBit + select)){
      succeed = false;
      ROS_INFO("fail to inject");
    }
    else{
      ROS_INFO("succeed write reg");
      record.push_back(select);
    }
    success_select = false;
  }

  // Continue the execution.
  for (int i = 0; i < traced_process_id.size(); ++i){
    ptraceSafe(PTRACE_DETACH, traced_process_id[i], 0, 0);    
  }
  ROS_INFO("continue process");
  //dbg(2) << "PTRACE_CONT\n";

  return succeed;

}


void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
        std::ofstream outfile, outfile1, outfile2;
        outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/fault_injection.txt", std::ios_base::app);
        outfile << "succeed: " << succeed << "reg: " << modified_reg << "\n";
        outfile.close();

        outfile2.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/recompute_time.txt", std::ios_base::app);
        outfile2 << 0 << "\n";
        for (auto v:inject_time){
             outfile2 << v << "\n";}
        outfile2.close();

        ros::shutdown();
    }
    exit(0);
}

void sigCONTHandlerPrivate(int signo) {
    if (signo == SIGCONT) {
      raise(SIGCONT);
      waiting = false;
    }
    else{
      ROS_INFO("fail to continue ROSFI");
    }
}
void StartCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(msg->data >=1){
    ROS_INFO("ROSFI start!");
    waiting = false;
  }
  

}
// Environmental variables.
//char **Envp = nullptr;

int main(int argc, char **argv) 
{
  //Envp = envp;
  ros::init(argc, argv, "ROSFI");
  ros::Time start_ros, end_ros;
  ros::NodeHandle nh;
  signal(SIGINT, sigIntHandlerPrivate);
  signal(SIGCONT, sigCONTHandlerPrivate);
  ros::Publisher FIPID_pub = nh.advertise<std_msgs::Int32>("/FIPID", 1);
  //ros::Publisher target_algo_pub = nh.advertise<std_msgs::Int32>("/target_algo", 1);
  ros::Subscriber sub = nh.subscribe("/PID", 1, PIDCallback);
  ros::Subscriber start_sub = nh.subscribe("/start", 1, StartCallback);

  ros::Subscriber traced_process = nh.subscribe("/traced_process", 1, traced_processCallback);
  //ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("process_start");
  //ros::Rate loop_rate(10);

  //signal(SIGCONT, sigIntHandlerPrivate);
  //sleep(10);
  // target algorithm for fault injection (1:octomap, 2:motion_planner)
  //int target_algo;
  ros::param::get("/ROSFI/process_select", process_select);
  ros::param::get("/ROSFI/num_flips", num_flips);
  //ros::param::get("/ROSFI/target_algo", target_algo);
  //prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY);
  prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY);
  srand (time(NULL));
  uint8_t ins;
  uint8_t *IP;
  int status;
  struct user_regs_struct regs, regs_test;
  struct user_fpregs_struct FpRegs;
  int count = 0;
  bool sent = false;
  bool attach = false;
  int waiting_time = 0;
  std::vector<std::string> v;
  while (ros::ok())
  {
    std_msgs::Int32 msg;
    std_msgs::Int32 start_msg;

    if(!sent){
      start_ros = ros::Time::now();
      msg.data = getpid();
      ROS_INFO("ROSFI id = %d ", msg.data);
      FIPID_pub.publish(msg);
      //msg.data = target_algo;
      //target_algo_pub.publish(msg);
      sent = true;
    }
    /*if(!ros::master::getNodes(v))
    {
      ROS_INFO("Error");

    }   
    else
    {  
      ROS_INFO("%s\n", v.size()); 
      for (int i = 0; i < v.size(); ++i)
      {
        ROS_INFO("%s\n", v[i]);
      }
    }*/
    if(FI_start && waiting == false && num_flips > 0){
      for (int i = 0; i < traced_process_id.size(); ++i){
        if(ptrace(PTRACE_SEIZE, traced_process_id[i], 0, 0) == -1){
          ROS_INFO("ptrace attach failed: %d\n", errno);
          if (errno == EBUSY){
            ROS_INFO("EBUSY");
          }
          else if (errno == EFAULT){
            ROS_INFO("EFAULT");
          }
          else if (errno == EINVAL){
            ROS_INFO("EINVAL");
          }
          else if(errno == EPERM){
            ROS_INFO("EPERM");
          }
          else if(errno == EIO){
            ROS_INFO("EIO");
          }
          else if(errno == ESRCH){
            ROS_INFO("ESRCH");
          }
        }
        else{
          ROS_INFO("attach success");
        }
      }

      //waiting_time = rand() % 5000; //+ rand() % (700 - 100);
      waiting_time = 500;
      ROS_INFO("MAVFI wait %d us", waiting_time);

      usleep(waiting_time);
      //kill(traced_process_id[0], SIGCONT);
      start_ros = ros::Time::now();
      succeed = true;
      if(doBitFlip(num_flips)){
        end_ros = ros::Time::now();
        inject_time.push_back((end_ros - start_ros).toSec());
        std::cout << "total injection time: " <<(end_ros - start_ros).toSec() << "\n";
        ROS_INFO("MAVFI finish");
      }
      else{
        for (int i = 0; i < traced_process_id.size(); ++i){
          ptraceSafe(PTRACE_DETACH, traced_process_id[i], 0, 0);
        }
        ROS_INFO("injection fail");
        succeed = false;
      }
      FI_start = false;
      count++;
      
    }
    ros::spinOnce();
    //loop_rate.sleep();
  }


  return 0;
}
