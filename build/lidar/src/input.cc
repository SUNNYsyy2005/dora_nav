#include "../include/lslidar_driver/input.h"
#include <chrono>
#include <csignal>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <poll.h>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>

extern volatile sig_atomic_t flag;

namespace lslidar_driver {
//static const size_t packet_size = sizeof(lslidar_driver::LslidarPacket().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.https://github.com/dora-rs/autoware.universe/blob/feature/autoware_dora/dora-hardware/vendors/lidar/rslidar_driver_pcap.cc
 */
    Input::Input(Ros::NodeHandle private_nh, uint16_t port, int packet_size) : private_nh_(private_nh), port_(port),
                                                                               packet_size_(packet_size) {
        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;
        private_nh.param("device_ip", devip_str_, std::string(""));
        private_nh.param("add_multicast", add_multicast, false);
        private_nh.param("group_ip", group_ip, std::string("224.1.1.2"));
        //if (!devip_str_.empty())
            //ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
    }


////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(Ros::NodeHandle private_nh, uint16_t port, int packet_size) : Input(private_nh, port,
                                                                                                 packet_size) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }
        printf("Opening UDP socket port: %d\n", port);
        //ROS_INFO_STREAM("Opening UDP socket port: " << port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            std::cerr << "socket error" << std::endl;
            //perror("socket");  // TODO: ROS_ERROR errno
            return;
        }

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *) &opt, sizeof(opt))) {
            std::cerr << "setsockopt error!" << std::endl;
            //perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                   // my address information
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
        my_addr.sin_family = AF_INET;          // host byte order
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

        if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
            std::cerr << "bind error" << std::endl;
            //perror("bind");  // TODO: ROS_ERROR errno
            return;
        }
        if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            //group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                std::cerr << "Adding multicast group error" << std::endl;
                //perror("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
            std::cerr << "non-block error" << std::endl;
            //perror("non-block");
            return;
        }
    }


/** @brief destructor */
    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

/** @brief Get one lslidar packet. */
    int InputSocket::getPacket(lslidar_cx_driver::LslidarPacketPtr &pkt) {
        pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN; //有数据可读
        fds[0].revents = 0;
        printf("sockfd_:%d\n", sockfd_);
        printf("events:%d\n", fds[0].events);

        static const int POLL_TIMEOUT = 3000;  // one second (in msec)
        printf("POLL_TIMEOUT:%d\n", POLL_TIMEOUT);
        sockaddr_in sender_address{};   //存储发送者的地址信息
        socklen_t sender_address_len = sizeof(sender_address);
        printf("getPacket\n");
        while (flag == 1)
        {
            // Receive packets that should now be available from the
            // socket using a blocking read.
            // poll() until input available
            do {
                printf("poll begin\n");
                
                int retval = poll(fds, 1, POLL_TIMEOUT);
                
                if (retval < 0)  // poll() error?
                {
                    if (errno != EINTR){
                        std::cerr << "poll() error: " << strerror(errno) << std::endl;
                    }
                        //ROS_ERROR("poll() error: %s", strerror(errno));
                    return 1;
                }
                if (retval == 0)  // poll() timeout?
                {
                    time_t curTime = std::time(NULL);
                    struct tm *curTm = localtime(&curTime);
                    char bufTime[72] = {0};
                    sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
                            curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
                    std::cerr << bufTime << "  lslidar poll() timeout, port:" << port_ << std::endl;
                    //ROS_WARN("%s  lslidar poll() timeout, port:%d", bufTime, port_);
                    
                    return 1;
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
                    (fds[0].revents & POLLNVAL))  // device error?
                {
                    std::cerr << "poll() reports lslidar error" << std::endl;
                    //ROS_ERROR("poll() reports lslidar error");
                    return 1;
                }
                printf("poll end\n");
            } while ((fds[0].revents & POLLIN) == 0);   //从socket读取数据，将数据存储到pkt的数据部分中
            printf("recvfrom begin\n");
            ssize_t nbytes = recvfrom(sockfd_, pkt->data.data(), packet_size_, 0, (sockaddr *) &sender_address,
                                      &sender_address_len);

            if (nbytes < 0) {
                if (errno != EWOULDBLOCK) {
                    perror("recvfail");
                    //ROS_INFO("recvfail");
                    return 1;
                }
            } else if ((size_t) nbytes == packet_size_ || (int) nbytes == 1206) {
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) {
                    char sender_ip[INET_ADDRSTRLEN];
                    char dev_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &(sender_address.sin_addr), sender_ip, INET_ADDRSTRLEN);
                    inet_ntop(AF_INET, &(devip_), dev_ip, INET_ADDRSTRLEN);
                    std::cerr << devip_str_ << " " << dev_ip << " " << sender_ip << std::endl;
                    std::cerr << "sender ip address does not match the device ip address" << std::endl;
                    //ROS_WARN_THROTTLE(2, "lidar ip parameter error, please reset lidar ip in launch file");
                    continue;
                } else
                    break;  // done
            }

            //ROS_DEBUG_STREAM("incomplete lslidar packet read: " << nbytes << " bytes");
        }
        printf("getPacket end\n");
        if (flag == 0) {
            abort();
        }
        printf("flag not zero\n");
        // Average the times at which we begin and end reading.  Use that to
        // estimate when the scan occurred. Add the time offset.

        return 0;
    }



////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
    InputPCAP::InputPCAP(Ros::NodeHandle private_nh, uint16_t port, int packet_size, double packet_rate,
                         std::string filename,
                         bool read_once, bool read_fast, double repeat_delay)
            : Input(private_nh, port, packet_size), packet_rate_(packet_rate), filename_(filename) {
        pcap_ = NULL;
        empty_ = true;
        std::cout<<"filename:"<<filename_<<std::endl;
        // get parameters using private node handle
        private_nh.param("read_once", read_once_, false);
        private_nh.param("read_fast", read_fast_, false);
        private_nh.param("repeat_delay", repeat_delay_, 0.0);

        if (read_once_)
            printf("Read input file only once.");
        if (read_fast_)
            printf("Read input file as quickly as possible.");
        if (repeat_delay_ > 0.0)
            printf("Delay %.3f seconds before repeating input file.", repeat_delay_);

        // Open the PCAP dump file
        // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
        printf("Opening PCAP file \"%s\"\n", filename_.c_str());
        //ROS_INFO_STREAM("Opening PCAP file " << filename_);
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
            printf("Error opening lslidar socket dump file.");
            //ROS_FATAL("Error opening lslidar socket dump file.");
            return;
        }

        std::stringstream filter;
        if (devip_str_ != "")  // using specific IP?
        {
            filter << "src host " << devip_str_ << " && ";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

/** destructor */
    InputPCAP::~InputPCAP(void) {
        pcap_close(pcap_);
    }

/** @brief Get one lslidar packet. */
    int InputPCAP::getPacket(lslidar_cx_driver::LslidarPacketPtr &pkt) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;

        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
                // Skip packets not for the correct port and from the
                // selected IP address.
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
                    continue;

                // Keep the reader from blowing through the file.
                if (read_fast_ == false)
                    packet_rate_.sleep();


                memcpy(&pkt->data[0], pkt_data + 42, packet_size_);

                if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 &&
                    pkt->data[3] == 0x5A) {//difop
                    int rpm = (pkt->data[8] << 8) | pkt->data[9];
                    printf("lidar rpm:%d\n", rpm);
                    //ROS_DEBUG("lidar rpm:%d", rpm);
                }
                
                pkt->stamp = Ros::Time::now();  // time_offset not considered here, as no
                // synchronization required
                empty_ = false;
                return 0;  // success
            }

            if (empty_)  // no data in file?
            {
                printf("Error %d reading lslidar packet: %s\n", res, pcap_geterr(pcap_));
                //ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
                return -1;
            }

            if (read_once_) {
                printf("end of file reached -- done reading.\n");
                //ROS_INFO("end of file reached -- done reading.");
                return -1;
            }

            if (repeat_delay_ > 0.0) {
                printf("end of file reached -- delaying %.3f seconds.\n", repeat_delay_);
                //ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            printf("replaying lslidar dump file\n");
            //ROS_DEBUG("replaying lslidar dump file");

            // I can't figure out how to rewind the file, because it
            // starts with some kind of header.  So, close the file
            // and reopen it with pcap.
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;  // maybe the file disappeared?
        }                 // loop back and try again

        if (flag == 0) {
            abort();
        }

        return 0;
    }

} //namespace
