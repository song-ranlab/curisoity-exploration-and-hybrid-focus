#include <ctime>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <boost/asio.hpp>
#include <rost/io.hpp>
using boost::asio::ip::tcp;
using namespace std;
class TcpMessageBroadcaster{

public:
  int port;
  int Q_max_size;
  deque<string> messageQ;
  mutex Q_mutex;
  bool stop;
  std::thread network_thread;
  TcpMessageBroadcaster(int port_, int Q_size_):port(port_), Q_max_size(Q_size_),stop(false){    

    network_thread = std::thread(&TcpMessageBroadcaster::run,this);  
  }
  void push(const string& msg){
    lock_guard<mutex> lock(Q_mutex);
    messageQ.push_front(msg);
    while(messageQ.size() > Q_max_size)
      messageQ.pop_back();
  }
  

  void push(int time, cv::Mat& mat, string name){
    string msg=" ";
    msg+=to_string(time);
    msg+=":";
    switch(mat.type()){
    case CV_32FC1:  msg+=to_string_iter(mat.begin<float>(), mat.end<float>()); break;
    default: msg+="unsupported";
    }
    msg+=" ";
    msg+="\n";
    push(msg);
  }


  bool run ()
  {
    try
      {
	boost::asio::io_service io_service;

	tcp::endpoint endpoint(tcp::v4(), port);
	tcp::acceptor acceptor(io_service, endpoint);

	while (!stop)
	  {
	    tcp::iostream stream;
	    boost::system::error_code ec;
	    acceptor.accept(*stream.rdbuf(), ec);
	    if (!ec)
	      {
		do{
		  while(!messageQ.empty() && !stop){
		    stream << messageQ.back();
		    stream.flush();
		    {
		      lock_guard<mutex> lock(Q_mutex);
		      messageQ.pop_back(); 
		    }
		  }
		  this_thread::sleep_for(chrono::milliseconds( 10 ));
		}while(stream);
		cerr << "Connection Error:" << stream.error().message() << endl;
	      }
	  }
	if(stop){
	  cerr<<"Stoping TCP broadcast"<<endl;
	  return true;
	}
      }
    catch (std::exception& e)
      {
	std::cerr << e.what() << std::endl;
      }
    return false;
  }
};
