#include "rgbd_image.h"

void RGBDIMAGE::getAssociationFile(const std::string& dataset_name, std::vector<std::string>& rgb_assoc_vec,std::vector<std::string>& depth_assoc_vec, std::vector<std::string>& truth_assoc_vec){
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

void RGBDIMAGE::dataSyncronize(const std::string& dataset_name, std::vector<std::string>& rgb_name_sync, std::vector<std::string>& depth_name_sync, std::vector<double>& time_sync){
  std::cout<<" TUM Data synchronization ... "<<std::endl;

  char* temp;
  std::vector<std::string> rgb_time_str,depth_time_str;
  std::vector<double> rgb_time, rgb_time_gap, depth_time, depth_time_gap;
  std::vector<std::string>::const_iterator rgb_iter,depth_iter; // const_iterator is needed !!
  std::vector<std::string> rgb_name_raw, depth_name_raw, truth_assoc_vec;
  std::vector<double> t_cam_vec;

  RGBDIMAGE::getAssociationFile(dataset_name, rgb_name_raw, depth_name_raw, truth_assoc_vec);

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

void RGBDIMAGE::getImage(const std::string& img_name, const std::string& depth_name, const double& scale_, cv::Mat& img_o, cv::Mat& depth_o){

  img_o = cv::imread(img_name,0); // 0 : gray image, -1 : intact image
  depth_o = cv::imread(depth_name,-1);
  depth_o.convertTo(depth_o,CV_64F);
  depth_o*=scale_;
  //output : img_o ( CV_8U = uchar ), depth_o ( CV_64F = double )
}

void RGBDIMAGE::downSampleImage(cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(cv::Size(img_i.cols / 2, img_i.rows / 2), img_i.type());
    int u2,u21;
    for(int v = 0; v < img_o.rows; ++v) {
        uchar* img_i_row_ptr1 = img_i.ptr<uchar>(2*v);
        uchar* img_i_row_ptr2 = img_i.ptr<uchar>(2*v+1);
        uchar* img_o_row_ptr = img_o.ptr<uchar>(v);
        for(int u = 0; u < img_o.cols; ++u) {
          u2 = 2*u;
          u21 = u2+1;
          img_o_row_ptr[u] =  (uchar)((img_i_row_ptr1[u2] + img_i_row_ptr1[u21] + img_i_row_ptr2[u2] + img_i_row_ptr2[u21]) / 4.0 );
        }
    }
}

void RGBDIMAGE::downSampleDepth(cv::Mat& img_i, cv::Mat& img_o) {
    img_o.create(cv::Size(img_i.size().width / 2, img_i.size().height / 2), img_i.type());
    int u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    double sum, cnt;
    for(int v = 0; v < img_o.rows; ++v) {
        for(int u = 0; u < img_o.cols; ++u) {
            u0 = u * 2;
            u1 = u0 + 1;
            v0 = v * 2;
            v1 = v0 + 1;

            // initialize
            sum = 0;
            cnt = 0;

            if((img_i.at<double>(v0, u0) > 0.01f)) {
                sum += img_i.at<double>(v0, u0);
                cnt += 1;
            }
            if((img_i.at<double>(v0, u1) > 0.01f)) {
                sum += img_i.at<double>(v0, u1);
                cnt += 1;
            }
            if((img_i.at<double>(v1, u0) > 0.01f)) {
                sum += img_i.at<double>(v1, u0);
                cnt += 1;
            }
            if((img_i.at<double>(v1, u1) > 0.01f)) {
                sum += img_i.at<double>(v1, u1);
                cnt += 1;
            }
            if(cnt > 0) img_o.at<double>(v, u) = ( sum / cnt );
            else img_o.at<double>(v, u) = 0;
        }
    }
}

void RGBDIMAGE::findCannyPixels(cv::Mat& img_i){
  int cnt=0;
  for(int v=0; v<img_i.rows;v++){
    uchar* row_ptr = img_i.ptr<uchar>(v);
    for(int u=0; u<img_i.cols;u++){
      if(*(row_ptr++)>0){
        cnt++;
      }
    }
  }
  std::cout<<"cnt : "<<cnt<<std::endl;
}

void RGBDIMAGE::calcDerivX(cv::Mat& img_i, cv::Mat& img_o) {
  cv::Mat temp;
  temp.create(img_i.size(),CV_64F); // double
  int prev = 0, next = 0;

  for(int v = 0; v < img_i.rows; ++v) {
    uchar* input_ptr = img_i.ptr<uchar>(v);
    double* temp_ptr = temp.ptr<double>(v);
    for(int u = 1; u < img_i.cols-1; ++u) {
        *(temp_ptr+u) = (double)( *(input_ptr+1+u) - *(input_ptr-1+u) ) * 0.5;
    }
  }
  temp.copyTo(img_o);
}

void RGBDIMAGE::calcDerivY( cv::Mat& img_i, cv::Mat& img_o) {
  cv::Mat temp;
  temp.create(img_i.size(),CV_64F);
  for(int v = 1; v < img_i.rows-1; ++v) {
    uchar* input_ptr_low = img_i.ptr<uchar>(v-1);
    uchar* input_ptr_high = img_i.ptr<uchar>(v+1);
    double* temp_ptr = temp.ptr<double>(v);
    for(int u = 0; u < img_i.cols; ++u) {
      *(temp_ptr+u) = (double)( *(input_ptr_high+u) - *(input_ptr_low+u) ) * 0.5;
    }
  }
  temp.copyTo(img_o);
}

void RGBDIMAGE::calcDerivNorm(cv::Mat& dx, cv::Mat& dy, cv::Mat& img_o) {
  cv::Mat temp;
  temp.create(dx.size(),CV_64F);
  for(int v = 0; v< dx.rows;v++){
    double* dx_ptr = dx.ptr<double>(v);
    double* dy_ptr = dy.ptr<double>(v);
    double* temp_ptr = temp.ptr<double>(v);
    for(int u=0;u<dx.cols;u++){
      *(temp_ptr+u) = sqrt((*(dx_ptr+u))*(*(dx_ptr+u)) + (*(dy_ptr+u))*(*(dy_ptr+u)) );
    }
  }
  temp.copyTo(img_o);
}

void RGBDIMAGE::calcDerivNorm(cv::Mat& dx, cv::Mat& dy, cv::Mat& img_o, cv::Mat& dx_o, cv::Mat& dy_o) {
  cv::Mat temp, temp_x, temp_y;
  temp.create(dx.size(),CV_64F);
  temp_x.create(dx.size(),CV_64F);
  temp_y.create(dx.size(),CV_64F);

  for(int v = 0; v< dx.rows;v++){
    double* dx_ptr = dx.ptr<double>(v);
    double* dy_ptr = dy.ptr<double>(v);
    double* temp_x_ptr = temp_x.ptr<double>(v);
    double* temp_y_ptr = temp_y.ptr<double>(v);
    double* temp_ptr = temp.ptr<double>(v);
    for(int u=0;u<dx.cols;u++){
      *(temp_ptr+u) = sqrt((*(dx_ptr+u))*(*(dx_ptr+u)) + (*(dy_ptr+u))*(*(dy_ptr+u)) );
      *(temp_x_ptr+u) = (*(dx_ptr+u)) / (*(temp_ptr+u));
      *(temp_y_ptr+u) = (*(dy_ptr+u)) / (*(temp_ptr+u));
    }
  }
  temp.copyTo(img_o);
  temp_x.copyTo(dx_o);
  temp_y.copyTo(dy_o);
}

void RGBDIMAGE::findValidMask(cv::Mat& edge_map, cv::Mat& depth_map, cv::Mat& img_o, int& valid_num_px){
  cv::Mat temp;
  valid_num_px = 0; // initialize
  temp.create(edge_map.size(),CV_8U);
  for(int v = 0; v< edge_map.rows;v++){
    uchar* edge_ptr = edge_map.ptr<uchar>(v);
    double* depth_ptr = depth_map.ptr<double>(v);
    uchar* temp_ptr = temp.ptr<uchar>(v);

    for(int u = 0; u<edge_map.cols; u++){
      if(*(edge_ptr++) > 0 & *(depth_ptr++) > 0){
        *(temp_ptr++) = 255;
        valid_num_px++;
      }
      else{
        *(temp_ptr++) = 0;
      }
    }
  }
  temp.copyTo(img_o); // output : valid pixel mask cv::Mat image
}

void RGBDIMAGE::setEdgePoints(cv::Mat& valid_mask, cv::Mat& grad_x, cv::Mat& grad_y, int& valid_num, std::vector<double>& arr_u, std::vector<double>& arr_v, std::vector<double>& arr_grad_u, std::vector<double>&arr_grad_v){//input : valid mask(Mat), ptr (Eigen::data()) row major
  arr_u.resize(0); // initialize
  arr_v.resize(0);
  arr_grad_u.resize(0);
  arr_grad_v.resize(0);
  int ind = 0;
  for(int v = 0;v<valid_mask.rows;v++){
    uchar* valid_mask_ptr = valid_mask.ptr<uchar>(v);
    double* grad_x_ptr = grad_x.ptr<double>(v);
    double* grad_y_ptr = grad_y.ptr<double>(v);

    for(int u = 0; u<valid_mask.cols;u++){
    //  std::cout<<(int)*(valid_mask_ptr++)<<std::endl;
      if(*(valid_mask_ptr++)==255){
      //  std::cout<<"in " <<u<<" , "<<v<<std::endl;
        arr_u.push_back(u);
        arr_v.push_back(v);
        arr_grad_u.push_back(*(grad_x_ptr+u));
        arr_grad_v.push_back(*(grad_y_ptr+u));
        //arr_grad_v(ind)=*(grad_y_ptr+u);
        ind++;
      }
    }
  }
}




// not use

void RGBDIMAGE::calcDerivX2(cv::Mat& img_i, cv::Mat& img_o) { // TOO SLOW ! SLOWER than ptr access
    img_o.create(img_i.size(), CV_64F);
    int prev = 0, next = 0;
    for(int y = 0; y < img_i.rows; ++y) {
        for(int x = 0; x < img_i.cols; ++x) {
            prev = std::max(x - 1, 0);
            next = std::min(x + 1, img_i.cols - 1);
            img_o.at<double>(y, x) = ( (img_i.at<uchar>(y, next) - img_i.at<uchar>(y, prev)) * 0.5f );
        }
    }
}

void RGBDIMAGE::calcDerivY2(cv::Mat& img_i, cv::Mat& img_o) { // TOO SLOW ! SLOWER than ptr access
    img_o.create(img_i.size(), CV_64F);
    int prev = 0, next = 0;
    for(int y = 0; y < img_i.rows; ++y) {
        for(int x = 0; x < img_i.cols; ++x) {
            prev = std::max(y - 1, 0);
            next = std::min(y + 1, img_i.rows - 1);

            img_o.at<double>(y, x) = ((img_i.at<uchar>(next, x) - img_i.at<uchar>(prev, x)) * 0.5f );
        }
    }
}

void RGBDIMAGE::downSampleImage2(cv::Mat& img_i, cv::Mat& img_o) {
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

void RGBDIMAGE::dummyFunc(){
  Eigen::MatrixXd Jx = Eigen::MatrixXd::Random(5000,6);
  Eigen::MatrixXd rx = Eigen::MatrixXd::Random(5000,1);
  Eigen::MatrixXd Jy = Eigen::MatrixXd::Random(5000,6);
  Eigen::MatrixXd ry = Eigen::MatrixXd::Random(5000,1);
  Eigen::MatrixXd res,Hx,Hy;

  Eigen::Matrix<double,6,6> HH = Eigen::MatrixXd(6,6);

  for(int k=0;k<30;k++){
    //Hx = Jx.transpose()*Jx;
    //Hy = Jy.transpose()*Jy;
    //res = Hx.inverse()*Jx.transpose()*rx+Hy.inverse()*Jy.transpose()*ry;
    res=(Jx.transpose()*Jx).inverse()*Jx.transpose()*rx +(Jx.transpose()*Jx).inverse()*Jx.transpose()*rx ;
  }


}


/*double interp2(const cv::Mat& img_i, const double& u, const double& v) {
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
