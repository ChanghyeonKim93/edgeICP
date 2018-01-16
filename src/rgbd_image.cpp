#include "rgbd_image.h"

void getAssociationFile(const std::string& dataset_name, std::vector<std::string>& rgb_assoc_vec,std::vector<std::string>& depth_assoc_vec, std::vector<std::string>& truth_assoc_vec){
  std::cout<<"   Association file reading : "<<dataset_name<<" ... ";
  std::ifstream rgb_file("../dataset/"+dataset_name+"/rgb.txt");
  std::ifstream depth_file("../dataset/"+dataset_name+"/depth.txt");
  std::ifstream truth_file("../dataset/"+dataset_name+"/groundtruth.txt");
  std::string temp_str;

  while(std::getline(rgb_file,temp_str)){
    if(temp_str[0] !='#') rgb_assoc_vec.push_back(temp_str);
  }
  while(std::getline(depth_file,temp_str)){
    if(temp_str[0] !='#') depth_assoc_vec.push_back(temp_str);
  }
  while(std::getline(truth_file,temp_str)){
    if(temp_str[0] !='#') truth_assoc_vec.push_back(temp_str);
  }
  std::cout<<" DONE !"<<std::endl;
}

void dataSyncronize(const std::string& dataset_name, std::vector<std::string>& rgb_name_sync, std::vector<std::string>& depth_name_sync, std::vector<double>& time_sync){
  std::cout<<" TUM Data synchronization ... "<<std::endl;

  char* temp;
  std::vector<std::string> rgb_time_str,depth_time_str;
  std::vector<double> rgb_time, rgb_time_gap, depth_time, depth_time_gap;
  std::vector<std::string>::const_iterator rgb_iter,depth_iter; // const_iterator is needed !!
  std::vector<std::string> rgb_name_raw, depth_name_raw, truth_assoc_vec;
  std::vector<double> t_cam_vec;

  getAssociationFile(dataset_name, rgb_name_raw, depth_name_raw, truth_assoc_vec);

  // divide the time and filename
  for(rgb_iter = rgb_name_raw.begin(); rgb_iter!=rgb_name_raw.end(); rgb_iter++){
    temp = std::strtok((char*)(*rgb_iter).c_str()," ,");
    rgb_time_str.push_back(temp);
    rgb_time.push_back(std::stod(temp));
  }
  for(depth_iter = depth_name_raw.begin(); depth_iter!=depth_name_raw.end(); depth_iter++){
    temp = std::strtok((char*)(*depth_iter).c_str()," ,");
    depth_time_str.push_back(temp);
    depth_time.push_back(std::stod(temp));
  }
  // synchronizing
  int r_ind = 0, d_ind = 0, save_ind = 0;
  std::vector<int> r_flag, d_flag;
  double t_crit = 1.5*0.01; // 0.02 seconds
  int len = (int)rgb_name_raw.size()<=(int)depth_name_raw.size() ? rgb_name_raw.size() : depth_name_raw.size();

  while(r_ind <= len){
    if( (rgb_time[r_ind]-depth_time[d_ind]) >= t_crit )  d_ind+=1;
    else if( (rgb_time[r_ind]-depth_time[d_ind]) <= -t_crit)  r_ind+=1;
    else if( abs(rgb_time[r_ind]-depth_time[d_ind]) <= t_crit){
      rgb_name_sync.push_back("../dataset/"+dataset_name+"/rgb/"+rgb_time_str[r_ind]+".png");
      depth_name_sync.push_back("../dataset/"+dataset_name+"/depth/"+depth_time_str[d_ind]+".png");
      r_ind+=1;
      d_ind+=1;
    }
  }
  // done !
  std::cout<<" Synch length : "<<rgb_name_sync.size();
  std::cout<<" ... DONE ! "<<std::endl;
}

void getImage(const std::string& img_name, const std::string& depth_name, cv::Mat& img_o, cv::Mat& depth_o){

  img_o = cv::imread(img_name,-1); // -1 : intact image
  depth_o = cv::imread(depth_name,-1);
  //depth_o.convertTo(depth_o,CV_16U);
}

void downSampleImage(cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(cv::Size(img_i.cols / 2, img_i.rows / 2), img_i.type());
    int u2,u21;
    for(int v = 0; v < img_o.rows; ++v) {
        double* img_i_row_ptr1 = img_i.ptr<double>(2*v);
        double* img_i_row_ptr2 = img_i.ptr<double>(2*v+1);
        double* img_o_row_ptr = img_o.ptr<double>(v);
        for(int u = 0; u < img_o.cols; ++u) {
          u2 = 2*u;
          u21 = u2+1;

            img_o_row_ptr[u] =  (img_i_row_ptr1[u2] + img_i_row_ptr1[u21] + img_i_row_ptr2[u2] + img_i_row_ptr2[u21]) / 4.0 ;
        }
    }
}

void downSampleImage2(cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(cv::Size(img_i.cols / 2, img_i.rows / 2), img_i.type());
    int x0, x1, y0, y1;
  	for (int y=0;y<img_o.rows;++y){
  		for(int x=0;x<img_o.cols;++x){
  			x0 = x * 2;
  			x1 = x0 + 1;
  			y0 = y * 2;
  			y1 = y0 + 1;
  			img_o.at<double>(y,x) = (img_i.at<double>(y0,x0) + img_i.at<double>(y0,x1) + img_i.at<double>(y1,x0) + img_i.at<double>(y1,x1)) / 4.0;
  		}
  	}
}
void downSampleDepth(const cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(cv::Size(img_i.size().width / 2, img_i.size().height / 2), img_i.type());
    int x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    double sum, cnt;
    for(int y = 0; y < img_o.rows; ++y) {
        for(int x = 0; x < img_o.cols; ++x) {
            x0 = x * 2;
            x1 = x0 + 1;
            y0 = y * 2;
            y1 = y0 + 1;

            // initialize
            sum = 0;
            cnt = 0;

            if((img_i.at<double>(y0, x0) != 0.0f)) {
                sum += img_i.at<double>(y0, x0);
                cnt += 1;
            }
            if((img_i.at<double>(y0, x1) != 0.0f)) {
                sum += img_i.at<double>(y0, x1);
                cnt += 1;
            }
            if((img_i.at<double>(y1, x0) != 0.0f)) {
                sum += img_i.at<double>(y1, x0);
                cnt += 1;
            }
            if((img_i.at<double>(y1, x1) != 0.0f)) {
                sum += img_i.at<double>(y1, x1);
                cnt += 1;
            }
            if(cnt > 0) img_o.at<double>(y, x) = ( sum / cnt );
            else img_o.at<double>(y, x) = 0;
        }
    }
}

/*void calcDerivX(const cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(img_i.size(), img_i.type());
    int prev = 0, next = 0;
    for(int y = 0; y < img_i.rows; ++y) {
        for(int x = 0; x < img_i.cols; ++x) {
            prev = std::max(x - 1, 0);
            next = std::min(x + 1, img_i.cols - 1);
            img_o.at<double>(y, x) = ( (img_i.at<double>(y, next) - img_i.at<double>(y, prev)) * 0.5f );
        }
    }
}

void calcDerivY(const cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(img_i.size(), img_i.type());
    int prev = 0, next = 0;
    for(int y = 0; y < img_i.rows; ++y) {
        for(int x = 0; x < img_i.cols; ++x) {
            prev = std::max(y - 1, 0);
            next = std::min(y + 1, img_i.rows - 1);

            img_o.at<double>(y, x) = ( (img_i.at<double>(next, x) - img_i.at<double>(prev, x)) * 0.5f );
        }
    }
}

double interp2(const cv::Mat& img_i, const double& u, const double& v) {
    // 1-based pixel coordinates (u,v) and (x0, y0) / (x1, y1)
    int x0 = (int) (std::floor(u));
    int y0 = (int) (std::floor(v));
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    double x_weight = (u - x0);
    double y_weight = (v - y0);

    double interpolated =
        ( img_i.at<double>((y0-1), (x0-1)) * x_weight + img_i.at<double>((y0-1), (x1-1)) * (1 - x_weight) +
          img_i.at<double>((y1-1), (x0-1)) * x_weight + img_i.at<double>((y1-1), (x1-1)) * (1 - x_weight) +
          img_i.at<double>((y0-1), (x0-1)) * y_weight + img_i.at<double>((y1-1), (x0-1)) * (1 - y_weight) +
          img_i.at<double>((y0-1), (x1-1)) * y_weight + img_i.at<double>((y1-1), (x1-1)) * (1 - y_weight) );

    return (interpolated * 0.25f);
}*/
