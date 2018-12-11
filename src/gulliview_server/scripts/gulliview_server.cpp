//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
// Copyright (c) 2013-2014 Andrew Soderberg-Rivkin <sandrew@student.chalmers.se>
// Copyright (c) 2013-2014 Sanjana Hangal <sanjana@student.chalmers.se>
// Copyright (c) 2014 Thomas Petig <petig@chalmers.se>
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
//#include "OpenCVHelper.h"
//#include "TagDetector.h"

#include <sys/time.h>

#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
//#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ros/ros.h"
#include "custom_msgs/GulliViewPositions.h"
//#include "CameraUtil.h"


using namespace std;
//using helper::ImageSource;
using boost::asio::ip::udp;
using boost::posix_time::ptime;
using boost::posix_time::time_duration;
int main(int argc, char **argv)
{
   // ROS INIT
    ros::init(argc, argv, "gulliview");
    ros::NodeHandle n;

   // init pos
//   Pos *position = new Pos();
   custom_msgs::GulliViewPositions msg;
   ros::Publisher position_pub = n.advertise<custom_msgs::GulliViewPositions>("gv_positions", 1000);
   ros::Rate loop_rate(50);

   // Create logfile to be used
   // std::ofstream fout("GulliViewLog.txt");
   try {
      boost::asio::io_service io_service;

      udp::socket socket(io_service, udp::endpoint(udp::v4(), 2020));
      boost::array<uint8_t, 256> answer;

      boost::array<uint8_t, 256> recv_buf;
      while(ros::ok()) {
         udp::endpoint remote_endpoint;

         //boost::system::error_code error;
         socket.receive_from( boost::asio::buffer(recv_buf), remote_endpoint);
         ptime recvTime;
         recvTime = boost::posix_time::microsec_clock::local_time();
         uint32_t index = 0;
         uint32_t type    =  recv_buf[0] << 24 | recv_buf[1] << 16 | recv_buf[2] << 8 | recv_buf[3];
         uint32_t sub_type = recv_buf[4] << 24 | recv_buf[5] << 16 | recv_buf[6] << 8 | recv_buf[7];

         //recv_buf[index-1] = 1; //HACK: prepare for retransmission

         //New position data from the camera
         if (type == 1 and sub_type == 2) {
            uint32_t seq     = recv_buf[8] << 24 | recv_buf[9] << 16 | recv_buf[10] << 8 | recv_buf[11];
            index += 16;
            uint32_t len     = recv_buf[28] << 24 | recv_buf[29] << 16 | recv_buf[30] << 8 | recv_buf[31];
            std::cout << "seq " << seq << " len " << len << std::endl;
            int32_t nid =0;
            int32_t nx =0;
            int32_t ny =0;
            int32_t nt =0;
            int32_t id = 0;
            int32_t x = 0;
            int32_t y = 0;
            int32_t t = 0;
            for (size_t i = 0; i < len; ++i) {
               if(i==0){
                 id= recv_buf[32] << 24 | recv_buf[33] << 16 | recv_buf[34] << 8 | recv_buf[35];
                 x = recv_buf[36] << 24 | recv_buf[37] << 16 | recv_buf[38] << 8 | recv_buf[39];
                 y = recv_buf[40] << 24 | recv_buf[41] << 16 | recv_buf[42] << 8 | recv_buf[43];
                 t = recv_buf[44] << 24 | recv_buf[45] << 16 | recv_buf[46] << 8 | recv_buf[47];
               }else{
                 nid= recv_buf[48] << 24 | recv_buf[49] << 16 | recv_buf[50] << 8 | recv_buf[51];
                 nx = recv_buf[52] << 24 | recv_buf[53] << 16 | recv_buf[54] << 8 | recv_buf[55];
                 ny = recv_buf[56] << 24 | recv_buf[57] << 16 | recv_buf[58] << 8 | recv_buf[59];
                 nt = recv_buf[60] << 24 | recv_buf[61] << 16 | recv_buf[62] << 8 | recv_buf[63];

               }
               answer = recv_buf;
               msg.p1.x = x;
               msg.p1.y = y;
               msg.cameraid = t;
               msg.tagid1 = id;
               msg.p2.x = nx;
               msg.p2.y = ny;
               msg.tagid2 = nid;
               //std::cout << "tagid: " << msg.tagid;
               std::cout << "Tag: " << id << " x: " << x << " y: " << y << " heading: " << t << std::endl;
            }
            position_pub.publish(msg);
            //Data request from the Gulliver map client
         } else if (type == 1 and sub_type == 1) { //TODO: Correct types
             //Set the type to the response type
             answer[3] = 2;
             answer[7] = 2;
             udp::socket replysocket(io_service, udp::endpoint(udp::v4(), 8989));
             udp::endpoint remote_2 (remote_endpoint.address(),4242);
             replysocket.send_to(boost::asio::buffer(answer), remote_2);
         }
         //std::cout<< recvTime << "\n";
//      std::string data(recv_buf.begin(), recv_buf.end());
//      std::cout << "Data: " << data << "\n";
//      unsigned firstDel = data.find('[');
//      unsigned lastDel = data.find(']');
//      string strNew = data.substr (firstDel+1,(lastDel-firstDel)-1);
         //std::cout << strNew << "\n";
//      ptime startProcTime;
//      startProcTime = boost::lexical_cast<ptime>(strNew);
         //std::cout << startProcTime << "\n";
//      time_duration fullProcessTime = recvTime - startProcTime;
         //std::cout << "Full Proc Time: " <<fullProcessTime << "\n";
         // Time Stamp --- Received
         //std::cout << recv_buf.data() << "\n";
         // Write to logfile and save
//      fout << recv_buf.data() << "" << std::endl;

        ros::spinOnce();
      }
   } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
   }
/*   // Close log file
   fout << "Program closed: " << std::endl;
   fout.close(); */
   return 0;
}
