#include <visualwords/image_source.hpp>
#include <visualwords/texton_words.hpp>
#include <visualwords/feature_words.hpp>
#include <visualwords/color_words.hpp>
#include <visualize/draw_keypoints.hpp>
#include <visualize/draw_topic_hist.hpp>
#include <rost/program_options.hpp>
#include <rost/rost.hpp>
#include <rost/hlda.hpp>
#include <rost/io.hpp>
#include <future>
#include <boost/filesystem.hpp>
#include "broadcaster.hpp"
#include "sunshine_summarizer.hpp"
#include <fstream>

using namespace std;

#define POSEDIM 3
typedef array<int,POSEDIM> pose_t;
typedef neighbors<array<int,POSEDIM>> neighbors_t;
//typedef neighbors_hypercube<array<int,POSEDIM>> neighbors_t;
typedef hROST<pose_t, neighbors_t, hash_container<pose_t> > ROST_t;



pair< 	map<pose_t, vector<int>>,
		map<pose_t, vector<pose_t>> >
words_for_pose(WordObservation& z, int cell_size){
//	std::map<pose_t, vector<int>> m;
	map<pose_t, vector<int> > out_words;
	map<pose_t, vector<pose_t> > out_poses;

	for(size_t i=0;i<z.words.size(); ++i){
		pose_t pose, pose_original;
		pose[0]=z.seq;
		pose_original[0]=z.seq;
		for(size_t d=0; d<POSEDIM-1; ++d){
			pose[d+1]=z.word_pose[i*(POSEDIM-1)+d]/cell_size;
			pose_original[d+1]=z.word_pose[i*(POSEDIM-1)+d];
		}
		out_words[pose].push_back(z.words[i]);
		out_poses[pose].push_back(pose_original);
	}
	return make_pair(out_words,out_poses);
}

string  to_json(int time, cv::Mat& mat1, string name1, cv::Mat& mat2, string name2 )
{
  string msg= "";
  //msg+=",";
  switch(mat1.type()){
  case CV_32FC1:  msg+=to_string_iter(mat1.begin<float>(), mat1.end<float>()); break;
  default: msg+="unsupported";
  }
  msg+="]";


  //msg+="}\n";
  return msg;
}

WordObservation gaussian_filter(WordObservation& z, float sigma=100, int w=640, int h=480){
  assert((z.words.size()*2) == z.word_pose.size());
  WordObservation z_new=z;
  z_new.words.clear();
  z_new.word_pose.clear();
  z_new.word_scale.clear();
  std::random_device rd;
  std::mt19937 gen(rd());

  float sigma2=sigma*sigma;
  
  for(size_t i=0;i<z.words.size(); ++i){
    int x=z.word_pose[i*2], y=z.word_pose[i*2+1];
    float dx = z.word_pose[i*2] - w/2;
    float dy = z.word_pose[i*2+1] - h/2;   
    float p = exp( -(dx*dx + dy*dy)/sigma2);
    float r = generate_canonical<float,10>(gen);
    if(r<p){
      z_new.words.push_back(z.words[i]);
      z_new.word_scale.push_back(z.word_scale[i]);
      z_new.word_pose.push_back(x);
      z_new.word_pose.push_back(y);
    }
  }
  return z_new;
}

int main(int argc, char* argv[]){
  po::options_description options("Illuminates the world, information theoretically.");
  load_rost_base_options(options).add_options()
    ("gamma", po::value<double>()->default_value(0.001), "Rate of topic growth")
    ("hdp", "HDP for topic growth")
    ("delta", po::value<double>()->default_value(0), "How much to day dream")
    ("video", po::value<string>(),"Video file")
    ("image", po::value<string>(),"Image file")
    ("camera", po::value<int>(), "Use Camera with the given id. 0 => default source")    
    ("mjpgstream",po::value<string>(), "mjpgstream hostname")
    ("mjpgstream.path",po::value<string>()->default_value("/?action=stream"), "mjpeg stream GET path")
    ("mjpgstream.port",po::value<string>()->default_value("8080"), "mjpeg stream port")
    ("broadcaster.port", po::value<int>()->default_value(9001), "Port for the perplexity broadcaster 9001=> default")    
    ("camera.width", po::value<int>()->default_value(0), "Camera width 0=> default")    
    ("camera.height", po::value<int>()->default_value(0), "Camera height 0=> default")    
    ("rate", po::value<double>()->default_value(5.0), "Processing rate. (in frames per second)")    
    ("subsample", po::value<int>()->default_value(1),"Subsampling rate for input sequence of images and video.")
    ("scale", po::value<float>()->default_value(1.0),"Scale image")  
    ("g.sigma", po::value<double>()->default_value(0.5),"neighborhood decay (0,1] . how much attention to pay to the neighborhood. 1.0 => neighborhood cells are as important as the current cell.")  
    ("orb", po::value<bool>()->default_value(true),"Use ORB words.")  
    ("orb.num", po::value<int>()->default_value(1000),"Number of ORB words.")  
    ("texton", po::value<bool>()->default_value(true),"Use Texton words")  
    ("texton-cell-size", po::value<int>()->default_value(64),"Number of texto words in a column.")    
    ("color", po::value<bool>()->default_value(true),"Use Color words")  
    ("color-cell-size", po::value<int>()->default_value(32),"Number of color words in a column.")    
    ("topicmodel.update", po::value<bool>()->default_value(true), "Update global topic model with each iteration. only valid if a input topic model is given.")
    ("summarizer","Show summary")
    ("data-log", "Log relevant performance metrics")
    ("summarysize",po::value<int>()->default_value(6),"Summary size")
    ("save-imgs","Save images")
    ("fullscreen", "full screen vis")
    ("header",po::value<string>()->default_value("Sunshine"), "Header String")
    ("footer",po::value<string>()->default_value("Yogesh Girdhar"), "Footer string")
    ;
  auto args = read_command_line(argc, argv, options);

  std::ofstream data_log;
  if (args.count("data-log")) data_log = std::ofstream("sunshine_data.csv");

  char* data_root_c;
  data_root_c = getenv ("ROSTPATH");
  string data_root = string(ROSTPATH);
  if (data_root_c!=NULL){
    cerr<<"ROSTPATH: "<<data_root_c<<endl;
    data_root=data_root_c;
  }


  ImageSource * video_source;
  if(args.count("video")) 
    video_source = ImageSource::videofile(args["video"].as<string>());
  else if (args.count("camera")){
    video_source = ImageSource::camera(
				       args["camera"].as<int>(),
				       args["camera.width"].as<int>(),
				       args["camera.height"].as<int>()
				       );
  }
  else if (args.count("mjpgstream")){
    //video_source = new MJPGImageSource(args["mjpgstream"].as<string>(),"8080","/?action=stream");
    video_source = new MJPGImageSource(args["mjpgstream"].as<string>(),args["mjpgstream.port"].as<string>(),args["mjpgstream.path"].as<string>());
  }
  else if (args.count("image")) {
      video_source = new ImageFilenameSource(args["image"].as<string>());
  }
  else{
    throw std::invalid_argument("Must specify one of --video, --camera, --mjpgstream, or --image");
  }

  Subsample subsample(args["subsample"].as<int>());
  Scale scale(args["scale"].as<float>());
  *video_source > subsample > scale;
  ImagePipe * img_source =	&scale;

  string vocabname=data_root+"/libvisualwords/data/texton.vocabulary.baraka.1000.csv";
  TextonBOW texton_bow(0,args["texton-cell-size"].as<int>(), 1.0, vocabname);
  cerr<<"Done loading texton vocabulary"<<endl;
  vector<string> orb_feature_detector_names(1,string("ORB"));
  vector<int> orb_num_features(1,args["orb.num"].as<int>()) ;
  cerr<<"Initializing ORB BOW feature extractor"<<endl;
  //Add MSER feature detector
//  orb_feature_detector_names.push_back("MSER");  orb_num_features.push_back(0);
  LabFeatureBOW orb_bow(0,
             data_root+"/libvisualwords/data/orb_vocab/default.yml",
		     orb_feature_detector_names,
		     orb_num_features,
		     "ORB",
		     1.0);

  ColorBOW color_bow(0,args["color-cell-size"].as<int>(), 1.0,true,true);

  MultiBOW bow;
  if(args["texton"].as<bool>())
    bow.add(&texton_bow);
  if(args["orb"].as<bool>())
    bow.add(&orb_bow);
  if(args["color"].as<bool>())
    bow.add(&color_bow);

  cerr<<"Vocabulary size: "<<bow.vocabulary_size<<endl;
  int K=args["ntopics"].as<int>();
  int cell_size=args["cell.space"].as<int>();	
  double gamma=args["gamma"].as<double>();
  double alpha=args["alpha"].as<double>();
  double beta=args["beta"].as<double>();
  pose_t G;
  G[0]=args["g.time"].as<int>();
  G[1]=args["g.space"].as<int>();
  G[2]=args["g.space"].as<int>();
  ROST_t rost(bow.vocabulary_size,K,3,alpha,beta,gamma,neighbors_t(G));

  bool update_topic_model=true;
  if(args["in.topicmodel"].as<string>()!=""){
    //assert(false);
    cerr<<"Loading topic model: "<< args["in.topicmodel"].as<string>()<<endl;
    load_topic_model(rost,args["in.topicmodel"].as<string>());
    update_topic_model=args["topicmodel.update"].as<bool>(); 
  }

  SunshineSummarizer<ROST_t> summarizer(&rost, args["summarysize"].as<int>());

  atomic<bool> stop;
  stop.store(false);
  double tau=args["tau"].as<double>();
  int n_threads=args["threads"].as<int>();
  //auto workers =  parallel_refine_online2(&rost, tau,  0.5, 96 , n_threads, &stop);
  if(tau<0 || tau > 1) 
    tau = 0.5;
  //    auto workers =  parallel_refine_online_exp(&rost, tau , n_threads, &stop);
  auto workers =  parallel_refine_online_exp_beta(&rost, args["tau"].as<double>(),args["refine.weight.local"].as<double>(),args["refine.weight.global"].as<double>() , args["threads"].as<int>(), &stop);

  //    auto workers =  parallel_refine_online(&rost, tau, n_threads, &stop);


  cv::Mat img, img_gray;
  int i=0;
  vector<pose_t> current_poses, last_poses;
  map<pose_t, vector<pose_t>> last_word_poses;

  double rate=args["rate"].as<double>();
  int iter_process_time = static_cast<int>(1000/rate);
  int refine_count=0;
  std::chrono::time_point<std::chrono::system_clock> word_add_time, last_word_add_time=chrono::system_clock::now();

  //setup the tcp broadcaster 
  cerr<<"Starting perplexity broadcast on localhost:9001"<<endl;
  TcpMessageBroadcaster ppx_broadcaster(args["broadcaster.port"].as<int>(),4);
  //  auto broadcaster_result = std::async(std::launch::async, [&]{ppx_broadcaster.run();} );
  cerr<<"Outputting online perplexity to perplexity.json"<<endl;
  ofstream out_ppx_json("perplexity.json");
  if(args.count("save-imgs")){
    boost::filesystem::create_directory(boost::filesystem::path("images"));
  }
  //ofstream out_ppx_json("perplexity.json");

  bool show_topics=false, show_words=false, show_bw=false, show_color=false, show_wppx=false, show_tppx=false, show_plots=false, show_img=false;
  int ppx_show_type=2; string ppx_name="Topic+Word Perplexity";
  cv::Mat last_image;
  //cv::namedWindow("Perplexity", cv::WINDOW_NORMAL);
  //cv::loadWindowParameters("Perplexity");
  if(args.count("fullscreen")){
    cv::setWindowProperty("Perplexity", CV_WND_PROP_FULLSCREEN , CV_WINDOW_FULLSCREEN);  
  }

  auto font = cv::fontQt("Helvetica",-1,cv::Scalar(160,160,160));
  auto font_footer = cv::fontQt("Helvetica",8,cv::Scalar(160,160,160));

  while(true){
    img_source->grab();
    img_source->retrieve(img);
    //cv::cvtColor(img, img_gray, CV_BGR2GRAY);
    cerr<<"#img:"<<img.cols<<"x"<<img.rows<<"  ";
    if(args.count("save-imgs")){
      cv::imwrite(string("images/")+to_string(i)+".jpg",img);
    }
    WordObservation z = bow(img);
    //z = gaussian_filter(z,200);
    //cv::flip(img,img,1);
    z.seq=i++;
    cerr<<"#words:"<<z.words.size() << std::endl;
    if (i % 5 == 0) {
        rost.printHierarchy(std::cerr);
    }


    auto celldata = words_for_pose(z,cell_size);
    auto cellwords = celldata.first;
    auto wordposes = celldata.second;
    current_poses.clear();
    for(auto cell : cellwords){
      rost.add_observation(cell.first, cell.second.begin(), cell.second.end(), update_topic_model);
      current_poses.push_back(cell.first);
    }
		

    /// compute perplexity for the last pose
    /// this is done computing perplexity for each cell in the last time step
    /// and then averaging it.
    cerr<<"  #cells: "<<rost.C<<"("<<current_poses.size()<<")";
    WordObservation zz;
    zz.vocabulary_size=K;
    int ppx_rows=ceil((float)(img.rows)/cell_size), ppx_cols= ceil(float(img.cols)/cell_size);
    //    cerr<<"ppx rows,cols: "<<ppx_rows<<","<<ppx_cols<<endl;
    cv::Mat topic_ppx_cells(ppx_rows, ppx_cols, CV_32F);
    cv::Mat word_ppx_cells(ppx_rows, ppx_cols, CV_32F);
    topic_ppx_cells=0.0; word_ppx_cells=0.0;


    //TODO: remove tppx,wppx labels and replace them with local_ppx and global_ppx
    for(auto pose: last_poses){
      vector<int> topics = rost.get_ml_topics_for_pose(pose,true);
      auto cell = rost.get_cell(pose);
      float local_ppx_pose = rost.cell_perplexity_word(cell->W, rost.neighborhood(*cell));
      float global_ppx_pose = rost.cell_perplexity_word(cell->W, rost.get_weight_Z());

      //get the perplexity
      //      float tppx_pose=rost.topic_perplexity(pose);
      //      float wppx_pose=rost.perplexity(pose,false);
      float tppx_pose = global_ppx_pose;
      float wppx_pose = local_ppx_pose;

      //cerr<<"poses: "<<pose[2]<<","<<pose[1]<<endl;
      topic_ppx_cells.at<float>(pose[2],pose[1])=tppx_pose;
      word_ppx_cells.at<float>(pose[2],pose[1])=wppx_pose;

      //push the data int WordObservation struct, which is used for visualization
      vector<pose_t> wordposes = last_word_poses[pose];
      zz.words.insert(zz.words.end(),topics.begin(), topics.end());
      for(auto p: wordposes){
	for(size_t d=1; d<POSEDIM; ++d){
	  zz.word_pose.push_back(p[d]);
	}
	zz.word_scale.push_back(5);
      }
    }

    cv::Scalar mean_topic_ppx_s, stddev_topic_ppx_s, mean_word_ppx_s, stddev_word_ppx_s;
    cv::meanStdDev(topic_ppx_cells, mean_topic_ppx_s, stddev_topic_ppx_s);
    cv::meanStdDev(word_ppx_cells, mean_word_ppx_s, stddev_word_ppx_s);

    double 
      mean_topic_ppx=mean_topic_ppx_s.val[0], 
      mean_word_ppx=mean_word_ppx_s.val[0],
      std_topic_ppx=stddev_topic_ppx_s.val[0],
      std_word_ppx=stddev_word_ppx_s.val[0];

    string ppx_json =to_json(i-1, topic_ppx_cells,"topic_perplexity", word_ppx_cells,"word_perplexity");
    out_ppx_json<<ppx_json<<endl;
    //ppx_broadcaster.push(ppx_json);

    	/*
    cv::Mat summary_image;
    if(last_image.rows>0 && args.count("summarizer")){
      summarizer.add_timestep(i-1,last_image);
      summary_image=summarizer.render(800,600);
      cv::imshow("Summary",summary_image);
    }
	*/
    //last_image=img.clone();
      
    cerr<<" #W = "<<accumulate(rost.get_weight_Z().begin(), rost.get_weight_Z().end(),0);
    cerr<<" #TPPX = "<<mean_topic_ppx;
    cerr<<" #WPPX = "<<mean_word_ppx;
    cerr<<" #poses ="<<last_poses.size();

    cv::Mat words_out, topics_out;
    if(show_words){
      words_out= draw_keypoints(z,img,5);
      cv::imshow("Words",words_out);
    }

    if(show_img){
      cv::imshow("Source",img);
    }

    //visualize topics
    if(show_topics){
      cv::Mat blank(img.size(),img.type(),cv::Scalar(128,128,128));
      topics_out = draw_keypoints(zz,blank);
      cv::imshow("Topics",topics_out);
    }

    //visualize plots
    cv::Mat topicdist_plot(600,600,CV_8UC3,cv::Scalar(255,255,255));
    cv::Mat topicweight_plot(topicdist_plot,cv::Rect(0,0,600,300));
    cv::Mat topicweight_now_plot(topicdist_plot,cv::Rect(0,299,600,300));
    if(show_plots){
      draw_barchart(rost.get_weight_Z(),600,300,topicweight_plot);
      draw_barchart(calc_hist(zz.words),600,300,topicweight_now_plot);
      cv::imshow("Weights",topicdist_plot);
    }

    //visualize perplexity
    double max_ppx, min_ppx;
    cv::minMaxLoc(topic_ppx_cells,&min_ppx, &max_ppx);


    //normalize
    cv::Mat ppx_cells, ppx_plot_small(ppx_cells.rows, ppx_cells.cols, CV_8UC3, cv::Scalar(0,0,0)), ppx_plot;
    //string ppx_name;
    if(show_tppx & !show_wppx){
      ppx_cells = (topic_ppx_cells - mean_topic_ppx)/(2 * std_topic_ppx)+0.5 ;
      ppx_plot_small = colorize(ppx_cells, cv::Vec3b(0,0,255), cv::Vec3b(255,255,255));//show in red
    }
    else if(show_wppx & ! show_tppx){
      ppx_cells = (word_ppx_cells - mean_word_ppx)/(2 * std_word_ppx)+0.5 ;
      ppx_plot_small = colorize(ppx_cells, cv::Vec3b(0,0,255), cv::Vec3b(255,255,255)); // show in blue
    }
    else if(show_tppx && show_wppx){
      cv::multiply( ((topic_ppx_cells - mean_topic_ppx)/(2 * std_topic_ppx)+0.5), 
		    ((word_ppx_cells - mean_word_ppx)/(2 * std_word_ppx)+0.5) , ppx_cells);
      ;
      ppx_plot_small = colorize(ppx_cells, cv::Vec3b(0,0,255), cv::Vec3b(255,255,255)); //show in green
    }

    //cv::resize(ppx_plot_small,ppx_plot, img.size());
    //cv::addWeighted( img, 0.5, ppx_plot, 0.9, 0.0, ppx_plot);

    //footer
    //cv::addText(ppx_plot, "Curious Sunshine", cv::Point(ppx_plot.cols/2-100,12), font);
    //if(!args["header"].as<string>().empty())
       //cv::addText(ppx_plot, args["header"].as<string>(), cv::Point(10,40), font);

    //cv::addText(ppx_plot, "Yogesh Girdhar, Juan Camilo Gamboa, Travis Manderson, Greg Dudek", cv::Point(ppx_plot.cols/2-260,ppx_plot.rows-10), font);
    //if(!args["footer"].as<string>().empty())
       //cv::addText(ppx_plot, args["footer"].as<string>(), cv::Point(10,ppx_plot.rows-10), font_footer);
    
    //cv::imshow("Perplexity",ppx_plot);
    last_poses = current_poses;
    last_word_poses = wordposes;

    auto const new_refined = rost.get_refine_count() - refine_count;
    cerr<<"  #refine = "<< new_refined;
    if (args.count("data-log")) data_log << std::to_string(new_refined) << "\n";
    refine_count += new_refined;

    word_add_time = chrono::system_clock::now();
    int elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds> (word_add_time - last_word_add_time).count();
    int sleep_time = max<int>(10,iter_process_time-elapsed_time);
    int key=cv::waitKey(max<int>(100,iter_process_time-elapsed_time));
    last_word_add_time= chrono::system_clock::now();
    

    switch(key){
    case 'q': cv::saveWindowParameters("Perplexity"); goto done;			
    case 'f': 
      if(cv::getWindowProperty("Perplexity", CV_WND_PROP_FULLSCREEN) == CV_WINDOW_FULLSCREEN) 
	cv::setWindowProperty("Perplexity", CV_WND_PROP_FULLSCREEN , CV_WINDOW_NORMAL); 
      else
	cv::setWindowProperty("Perplexity", CV_WND_PROP_FULLSCREEN , CV_WINDOW_FULLSCREEN);  

      //cv::saveWindowParameters("Perplexity");
      break;

    case 'p': 
      ppx_show_type = (ppx_show_type+1) %3;
      if(ppx_show_type==0) {
	ppx_name="Topic Perplexity";
	show_tppx=true; show_wppx=false;

      }else if(ppx_show_type==1){
	ppx_name="Word Perplexity";
	show_tppx=false; show_wppx=true;
      }
      else{
	ppx_name="Topic+Word Perplexity";
	show_tppx=true; show_wppx=true;
      }
      cv::displayOverlay("Perplexity",ppx_name,5000);

      break;
    case 't': show_topics=!show_topics; break;
    case 'w': show_words=!show_words; break;
    case 'a': 
      cerr<<" alpha = "; 
      for(auto& a: rost.alpha){
	a*=1.5;	
	cerr<<" "<<a;
      } 
      cerr<<endl; 
      break;
    case 'z': 
      cerr<<" alpha = "; 
      for(auto& a: rost.alpha){
	a/=1.5;	
	cerr<<" "<<a;
      } 
      cerr<<endl; 
      break;
    case 's': 
      rost.beta *=1.5;	
      cerr<<" beta = "<<rost.beta<<endl; 
      break;
    case 'x': 
      rost.beta /=1.5;	
      cerr<<" beta = "<<rost.beta<<endl; 
      break;

    };
    cerr<<"        \r";
  }
 done:


  stop.store(true);
  ppx_broadcaster.stop=true;
  for(auto t:workers){
    t->join();
  }
  ppx_broadcaster.network_thread.detach();

  if(args["out.topicmodel"].as<string>()!=""){
    cerr<<"Saving topic model to:"<<args["out.topicmodel"].as<string>()<<endl;
    save_topic_model(rost,args["out.topicmodel"].as<string>());
    cerr<<"Done"<<endl;
  }

  //  write_topics(rost,args["out.topics"].as<string>(), wordposes, false);

  //broadcaster_result.get();
  // delete img_source;

  if (args.count("data-log")) data_log.close();

  return 0;
}
