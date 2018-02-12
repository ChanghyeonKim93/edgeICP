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
    short* dx_ptr = dx.ptr<short>(v);
    short* dy_ptr = dy.ptr<short>(v);
    double* temp_ptr = temp.ptr<double>(v);
    for(int u=0;u<dx.cols;u++){
      *(temp_ptr+u) = sqrt( (double)((*(dx_ptr+u))*(*(dx_ptr+u)) + (*(dy_ptr+u))*(*(dy_ptr+u))) );
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
    short* dx_ptr = dx.ptr<short>(v);
    short* dy_ptr = dy.ptr<short>(v);
    double* temp_x_ptr = temp_x.ptr<double>(v);
    double* temp_y_ptr = temp_y.ptr<double>(v);
    double* temp_ptr = temp.ptr<double>(v);
    for(int u=0;u<dx.cols;u++){
      *(temp_ptr+u) = sqrt( (double)((*(dx_ptr+u))*(*(dx_ptr+u)) + (double)(*(dy_ptr+u))*(*(dy_ptr+u))) );
      *(temp_x_ptr+u) = (double)(*(dx_ptr+u)) / (*(temp_ptr+u));
      *(temp_y_ptr+u) = (double)(*(dy_ptr+u)) / (*(temp_ptr+u));
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

void RGBDIMAGE::setEdgePoints(cv::Mat& valid_mask, cv::Mat& grad_x, cv::Mat& grad_y, cv::Mat& img_depth, std::vector<double>& arr_u, std::vector<double>& arr_v, std::vector<double>& arr_depth, std::vector<double>& arr_grad_u, std::vector<double>&arr_grad_v){//input : valid mask(Mat), ptr (Eigen::data()) row major
  arr_u.resize(0); // initialize
  arr_v.resize(0);
  arr_grad_u.resize(0);
  arr_grad_v.resize(0);
  arr_depth.resize(0);
  int ind = 0;
  for(int v = 0;v<valid_mask.rows;v++){
    uchar* valid_mask_ptr = valid_mask.ptr<uchar>(v);
    double* grad_x_ptr = grad_x.ptr<double>(v);
    double* grad_y_ptr = grad_y.ptr<double>(v);
    double* depth_ptr = img_depth.ptr<double>(v);

    for(int u = 0; u<valid_mask.cols;u++){
    //  std::cout<<(int)*(valid_mask_ptr++)<<std::endl;
      if(*(valid_mask_ptr++)==255){
      //  std::cout<<"in " <<u<<" , "<<v<<std::endl;
        arr_u.push_back(u);
        arr_v.push_back(v);
        arr_grad_u.push_back(*(grad_x_ptr+u));
        arr_grad_v.push_back(*(grad_y_ptr+u));
        arr_depth.push_back(*(depth_ptr+u));
        //arr_grad_v(ind)=*(grad_y_ptr+u);
        ind++;
      }
    }
  }
}

/*void RGBDIMAGE::warpPoints(const std::vector<Point_2d>& cur_edge_px_sub, const Eigen::MatrixXd& xi_temp){
  Eigen::Matrix4d g_temp;
  LIE::se3Exp(xi_temp,g_temp);

}
*/

void RGBDIMAGE::calcResidual(const std::vector<Point_4d>& key_edge_px_4d, const std::vector<Point_4d>& cur_edge_px_4d_sub, const std::vector<int>& ref_ind, std::vector<double>& res_x, std::vector<double>& res_y, std::vector<double>& residual){
  int N_sample = cur_edge_px_4d_sub.size();
  // resize
  residual.resize(N_sample);
  res_x.resize(N_sample);
  res_y.resize(N_sample);
  for(int i=0; i<N_sample; i++){
    res_x[i] = cur_edge_px_4d_sub[i][0] - key_edge_px_4d[ref_ind[i]][0];
    res_y[i] = cur_edge_px_4d_sub[i][1] - key_edge_px_4d[ref_ind[i]][1];
    residual[i] = res_x[i]*key_edge_px_4d[ref_ind[i]][0] + res_y[i]*key_edge_px_4d[ref_ind[i]][1]; // divergence around the edge pixels.
  }
}

void RGBDIMAGE::randsample(const int& npoints, const int& N_sample, std::vector<int>& sub_idx){
  std::vector<int> idx_vec(npoints,0);
  for(int i=0;i<npoints;i++) idx_vec[i]=i;

  std::random_shuffle(idx_vec.begin(),idx_vec.end());
  std::vector<int> null_vec;
  sub_idx.swap(null_vec);
  sub_idx.resize(N_sample);
  //sampling specific number of ...
  for(int i=0;i<N_sample;i++){
    sub_idx[i] = idx_vec[i];
    //std::cout<<idx_vec[i]<<", "; // for debug
  }
  //std::cout<<"input num : "<<npoints<<", debug - sub num : "<<sub_idx.size()<<std::endl;
}

void RGBDIMAGE::update_t_distribution(const std::vector<double>& residual, double& sigma){
  int nu = 5;
  int N = residual.size();
  double lambda_prev = 1.0/sigma/sigma;
  double temp=0.0, lambda_curr=0.0, sum=0.0;
  double eps = 0.0000001;
  while(1){
    for(int i=0;i<N;i++) sum+= residual[i]*residual[i] / ( (double)nu + lambda_prev*residual[i]*residual[i]);
    temp = ( (nu+1.0)/(double)N )*sum;

    if(fabs(lambda_curr-lambda_prev)<=eps) break;
    lambda_prev = lambda_curr;
  }
  sigma = sqrt(1.0/lambda_prev);
}

void RGBDIMAGE::calcJacobian(const std::vector<Point_2d>& warped_edge_px_sub, const std::vector<double>& warped_pt_depth, const std::vector<Point_4d>& key_edge_px_4d, const std::vector<int> ref_ind, const std::vector<double>& residual, const Eigen::Matrix3d& K, Eigen::MatrixXd& J){
  int n_points = warped_edge_px_sub.size();
  J = Eigen::MatrixXd::Zero(n_points,6); // need to initialize ?

  double fx=K(0,0),fy=K(1,1);
  double cx=K(0,2),cy=K(1,2);
  double g_x,g_y,X,Y,Z;
  double Zinv, XZinv, YZinv, fxgx, fygy;
  Point_3d temp3d(3,0);
  Point_3d temp3d_res(3,0);
  for(int i=0; i<n_points; i++){
    temp3d[0]=warped_edge_px_sub[i][0];
    temp3d[1]=warped_edge_px_sub[i][1];
    temp3d[2]=warped_pt_depth[i];
    proj_3d(temp3d,K,temp3d_res);
    X=temp3d_res[0];
    Y=temp3d_res[1];
    Z=temp3d_res[2]; // warped current pixels.
    g_x = key_edge_px_4d[ref_ind[i]][2];
    g_y = key_edge_px_4d[ref_ind[i]][3];
    Zinv = 1/Z;
    XZinv = X/Z;
    YZinv = Y/Z;
    fxgx = fx*g_x;
    fygy = fy*g_y;

    J(i,0)= Zinv*fxgx;
    J(i,1)= Zinv*fygy;
    J(i,2)= -XZinv*Zinv*fxgx - YZinv*Zinv*fygy;
    J(i,3)= -XZinv*YZinv*fxgx - (1+YZinv*YZinv)*fygy;
    J(i,4)= (1+XZinv*XZinv)*fxgx + XZinv*YZinv*fygy;
    J(i,5)= -YZinv*fxgx + XZinv*fygy;
  }
}

void RGBDIMAGE::proj_pixel(const Point_3d& input_arr, const Eigen::Matrix3d& K, Point_3d& output_arr){
  double fx = K(0,0), fy = K(1,1);
  double cx = K(0,2), cy = K(1,2);
  output_arr.resize(3,0);
  double u,v;
  double Zinv = 1/input_arr[2];
  u = fx*input_arr[0]*Zinv + cx;
  v = fy*input_arr[1]*Zinv + cy;
  output_arr[0]=u;
  output_arr[1]=v;
  output_arr[2]=1;
}

void RGBDIMAGE::proj_3d(const Point_3d& input_arr, const Eigen::Matrix3d& K, Point_3d& output_arr){
  double fx = K(0,0), fy = K(1,1);
  double cx = K(0,2), cy = K(1,2);
  double fxinv = 1/fx, fyinv = 1/fy;
  double X,Y,Z;
  output_arr.resize(3,0);
  Z = input_arr[2];
  X = (input_arr[0]-cx)*fxinv*Z;
  Y = (input_arr[1]-cy)*fyinv*Z;
  output_arr[0]=X;
  output_arr[1]=Y;
  output_arr[2]=Z;
}

void RGBDIMAGE::warpPoints(const std::vector<Point_4d>& cur_edge_px_4d_sub, const std::vector<double>& cur_pt_depth, const Eigen::Matrix3d& K, const Eigen::MatrixXd& xi_temp, std::vector<Point_2d>& warped_edge_px_sub, std::vector<Point_4d>& warped_edge_px_4d_sub, std::vector<double>& warped_pt_depth){
  Eigen::Matrix4d g_mat;
  int n_points = cur_edge_px_4d_sub.size();
  se3Exp(xi_temp, g_mat);
  Eigen::MatrixXd curr_points_affine(4,n_points);
  Eigen::MatrixXd warped_points_affine(4,n_points);

  for(int i=0;i<n_points;i++){
    Point_3d temp3d;
    Point_3d temp3d_res;

    temp3d.push_back(cur_edge_px_4d_sub[i][0]);
    temp3d.push_back(cur_edge_px_4d_sub[i][1]);
    temp3d.push_back(cur_pt_depth[i]);

    proj_3d(temp3d,K,temp3d_res);
    curr_points_affine(0,i) = temp3d_res[0];
    curr_points_affine(1,i) = temp3d_res[1];
    curr_points_affine(2,i) = temp3d_res[2];
    curr_points_affine(3,i) = 1.0;
  }

  // warping.
  warped_points_affine=g_mat*curr_points_affine;

  // reprojecting
  Point_2d temp2d(2,0);
  Point_4d temp4d(4,0);
  warped_edge_px_sub.resize(n_points,temp2d); // initialize
  warped_edge_px_4d_sub.resize(n_points,temp4d); // initialize
  warped_pt_depth.resize(n_points,0);
  for(int i=0;i<n_points;i++){
    Point_3d temp3d(3,0);
    Point_3d temp3d_res;
    temp3d[0] = warped_points_affine(0,i);
    temp3d[1] = warped_points_affine(1,i);
    temp3d[2] = warped_points_affine(2,i);
    warped_pt_depth[i]=  temp3d[2];
    proj_pixel(temp3d,K,temp3d_res);

    warped_edge_px_sub[i][0]    = temp3d_res[0];
    warped_edge_px_sub[i][1]    = temp3d_res[1];

    warped_edge_px_4d_sub[i][0] = temp3d_res[0];
    warped_edge_px_4d_sub[i][1] = temp3d_res[1];
    warped_edge_px_4d_sub[i][2] = cur_edge_px_4d_sub[i][2];
    warped_edge_px_4d_sub[i][3] = cur_edge_px_4d_sub[i][3];
  }
}


void RGBDIMAGE::se3Exp(const Eigen::MatrixXd& xi_temp, Eigen::Matrix4d& g){
  // initialize variables
  Eigen::Vector3d v, w;
  float length_w = 0.0;
  Eigen::Matrix3d Wx, R, V;
  Eigen::Vector3d t;

  v(0) = xi_temp(0);
  v(1) = xi_temp(1);
  v(2) = xi_temp(2);
  w(0) = xi_temp(3);
  w(1) = xi_temp(4);
  w(2) = xi_temp(5);

  length_w = std::sqrt(w.transpose() * w);
  hatOperator(w, Wx);
  if (length_w < 1e-7)
  {
      R = Eigen::Matrix3d::Identity(3,3) + Wx + 0.5 * Wx * Wx;
      V = Eigen::Matrix3d::Identity(3,3) + 0.5 * Wx + Wx * Wx / 3.0;
  }
  else
  {

      R = Eigen::Matrix3d::Identity(3,3) + (sin(length_w)/length_w) * Wx + ((1-cos(length_w))/(length_w*length_w)) * (Wx*Wx);
      V = Eigen::Matrix3d::Identity(3,3) + ((1-cos(length_w))/(length_w*length_w)) * Wx + ((length_w-sin(length_w))/(length_w*length_w*length_w)) * (Wx*Wx);
  }
  t = V * v;

  // assign rigid body transformation matrix (in SE(3))
  g = Eigen::MatrixXd::Identity(4,4);
  g(0,0) = R(0,0);
  g(0,1) = R(0,1);
  g(0,2) = R(0,2);

  g(1,0) = R(1,0);
  g(1,1) = R(1,1);
  g(1,2) = R(1,2);

  g(2,0) = R(2,0);
  g(2,1) = R(2,1);
  g(2,2) = R(2,2);

  g(0,3) = t(0);
  g(1,3) = t(1);
  g(2,3) = t(2);

      // for debug
      // std::cout << R << std::endl;
      // std::cout << t << std::endl;
      //usleep(10000000);
}

void RGBDIMAGE::hatOperator(const Eigen::Vector3d& col_vec, Eigen::Matrix3d& skew_mat){
   skew_mat(0,0) = 0;
   skew_mat(0,1) = -col_vec(2);
   skew_mat(0,2) = col_vec(1);

   skew_mat(1,0) = col_vec(2);
   skew_mat(1,1) = 0;
   skew_mat(1,2) = -col_vec(0);

   skew_mat(2,0) = -col_vec(1);
   skew_mat(2,1) = col_vec(0);
   skew_mat(2,2) = 0;
}


// not use

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
    res=(Jx.transpose()*Jx).inverse()*Jx.transpose()*rx +(Jx.transpose()*Jx).inverse()*Jx.transpose()*rx ;
    //res = Hx.inverse()*Jx.transpose()*rx+Hy.inverse()*Jy.transpose()*ry;
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
