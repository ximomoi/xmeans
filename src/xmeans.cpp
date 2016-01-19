#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <time.h>
#include <sstream>
#include <complex>
#include <stdlib.h>
#define k 2

using namespace Eigen;
using namespace std;

class Xmeans{
public:
  Xmeans(){
    dynamic_obstacle_sub = nh.subscribe("dynamic_obstacle", 100, &Xmeans::dynamic_obstacleCallBack, this);
    obstacle_clustering_pub = nh.advertise<sensor_msgs::PointCloud>("obstacle_clustering", 1000);
    objects_pub = nh.advertise<sensor_msgs::PointCloud>("objects", 1000);

    //cluster_divide_info(x, y, number, num_point_group, size, log_likelihood, check_ini, cov_det)
    cluster_divide_info = MatrixXf::Zero(8, k);

    callback = false;
  }

  void dynamic_obstacleCallBack(const sensor_msgs::PointCloud::ConstPtr& msg){
    int i, x_max = -30, y_max = 0;
    float add_coordinate_x = 0, add_coordinate_y = 0, x_ave_scalar, distance;
    callback = true;
    obstacle_num = (int)msg->points.size();
    cluster_num = 1;

    objects.header.frame_id = msg->header.frame_id;
    objects.header.stamp = ros::Time::now();

    clustering.header.frame_id = msg->header.frame_id;
    clustering.header.stamp = ros::Time::now();

    //cluster_info(x, y, number, num_point_group, size, log_likelihood, check, cov_det)
    cluster_info = MatrixXf::Zero(8, cluster_num);

    //obstacle_info(x, y, k_distance, cluster_divide_num, cluster_num)
    obstacle_info = MatrixXf::Zero(5, obstacle_num);

    clustering.points.resize(obstacle_num);
    clustering.channels.resize(1);

    ROS_INFO("-------------hogehoge---------------");
    for(i = 0; i < obstacle_num; i++){
      clustering.points[i].x = msg->points[i].x;
      clustering.points[i].y = msg->points[i].y;

      obstacle_info(0, i) = msg->points[i].x;
      obstacle_info(1, i) = msg->points[i].y;

      add_coordinate_x += obstacle_info(0, i);
      add_coordinate_y += obstacle_info(1, i);
      if(x_max < obstacle_info(0, i)){
        x_max = obstacle_info(0, i);
      }
      if(y_max < obstacle_info(1, i)){
        y_max = obstacle_info(1, i);
      }
    }
    cluster_info(0, 0) = add_coordinate_x / obstacle_num;
    cluster_info(1, 0) = add_coordinate_y / obstacle_num;
    cluster_info(3, 0) = obstacle_num;
    cluster_info(4, 0) = (x_max - cluster_info(0, 0)) * (y_max - cluster_info(1, 0)) * M_PI;

    //cov
    cov = Matrix2f::Zero();
    for(i = 0; i < obstacle_num; i++){
      cov(0, 0) += pow(cluster_info(0, 0) - obstacle_info(0, i), 2);
      cov(1, 1) += pow(cluster_info(1, 0) - obstacle_info(1, i), 2);
      cov(0, 1) += (cluster_info(0, 0) - obstacle_info(0, i)) * (cluster_info(1, 0) - obstacle_info(1, i));
    }
    cov(0, 0) = cov(0, 0) / obstacle_num;
    cov(1, 1) = cov(1, 1) / obstacle_num;
    cov(0, 1) = cov(0, 1) / obstacle_num;
    cov(1, 0) = cov(0, 1);

    cluster_info(7, 0) = cov.determinant();

    //確認
    ROS_INFO("cov_det:%f", cov.determinant());

    log_f = VectorXf::Zero(cluster_info(3, 0));
    x = Vector2f::Zero();
    ave = Vector2f::Zero();
    x_ave = Vector2f::Zero();
    ave(0) = cluster_info(0, 0);
    ave(1) = cluster_info(1, 0);
    for(i = 0; i < obstacle_num; i++){
        x(0) = obstacle_info(0, i);
        x(1) = obstacle_info(1, i);
        x_ave = x - ave;
        x_ave_scalar = x_ave.transpose() * cov.inverse() * x_ave;
        log_f(i) = (float)log((float)exp(- x_ave_scalar / 2) / (float)sqrt((float)pow(2 * M_PI, 2) * cov.determinant()));
    }

    cluster_info(5, 0) = log_f.sum();

    //確認
    ROS_INFO("log_likelihood:%f, x_ave_scalar:%f", cluster_info(5, 0), x_ave_scalar);
  }

  int check(){
    int count = 0, c;
    for(c = 0; c < cluster_num; c++){
      if(cluster_info(6, c) == 0){
        select_cluster = c;
        break;
      }
      count += 1;
    }
    if(cluster_num == count){
      return true;
    }else{
      return false;
    }
  }

  void kmeans(){
    int i, j, count, c;
    float deg_random, rad_random, range_random, distance, cluster_old_x[k], cluster_old_y[k], x_add, y_add;
    no_belong_cluster = false;

    //取得した障害物の座標を元にランダムクラスタを配置する
    srand((unsigned int)time(NULL));
    for(j = 0; j < k; j++){
      deg_random = (float)rand() / ((float)RAND_MAX + 1) * 360;
      rad_random = deg_random * M_PI / 180;
      range_random = (float)rand() / ((float)RAND_MAX + 1) * 0.1;
      cluster_divide_info(0, j) = range_random * cos(rad_random) + cluster_info(0, select_cluster);
      cluster_divide_info(1, j) = range_random * sin(rad_random) + cluster_info(1, select_cluster);
      cluster_divide_info(2, j) = j;

      //確認
      ROS_INFO("rand_clus_x:%f, rand_clus_y:%f", cluster_divide_info(0, j), cluster_divide_info(1, j));
    }

    while(true){
      int decision = 0;
      //クラスタの割り当て
      for(i = 0; i < obstacle_num; i++){
        if(cluster_info(2, select_cluster) == obstacle_info(4, i)){
          for(j = 0; j < k; j++){
            distance = hypotf(obstacle_info(0, i) - cluster_divide_info(0, j), obstacle_info(1, i) - cluster_divide_info(1, j));
            if(j == 0){
              obstacle_info(2, i) = distance;
              obstacle_info(3, i) = j;
            }else if(obstacle_info(2, i) > distance){
              obstacle_info(2, i) = distance;
              obstacle_info(3, i) = j;
            }
          }
        }
      }
      //確認
      ROS_INFO("1-----");
      ROS_INFO("range_rand:%f, deg_rand:%f", range_random, deg_random);
      ROS_INFO("------");

      //クラスタの移動
      for(j = 0; j < k; j++){
        cluster_old_x[j] = cluster_divide_info(0, j);
        cluster_old_y[j] = cluster_divide_info(1, j);

        x_add = 0;
        y_add = 0;
        count = 0;
        for(i = 0; i < obstacle_num; i++){
          if(cluster_info(2, select_cluster) == obstacle_info(4, i) && cluster_divide_info(2, j) == obstacle_info(3, i)){
            x_add += obstacle_info(0, i);
            y_add += obstacle_info(1, i);
            count += 1;
          }
        }
        cluster_divide_info(3, j) = count;

        if(count == 0){
          ROS_INFO("cluster_divide No:%d does not belong---------------------------------------------------------------", j);
          no_belong_cluster = true;
        }else{
          cluster_divide_info(0, j) = x_add / count;
          cluster_divide_info(1, j) = y_add / count;
        }
        //確認
        ROS_INFO("2-----");
        ROS_INFO("count:%d, k:%d", count, j);

        ROS_INFO("clus_old_x:%f, clus_old_y:%f", cluster_old_x[j], cluster_old_y[j]);
        ROS_INFO("         clus_x:%f,    clus_y:%f", cluster_divide_info(0, j), cluster_divide_info(1, j));
        ROS_INFO("------");
      }

      //今回のクラスタ座標と前回のクラスタ座標を比較
      for(j = 0; j < k; j++){
        if(cluster_divide_info(0, j) == cluster_old_x[j] && cluster_divide_info(1, j) == cluster_old_y[j]){
          decision += 1;
        }
      }
      if(decision == k){
        //size
        for(j = 0; j < k; j++){
          float max = 0, min = 60;
          for(i = 0; i < obstacle_num; i++){
            if(cluster_info(2, select_cluster) == obstacle_info(4, i) && cluster_divide_info(2, j) == obstacle_info(3, i)){
              if(max < obstacle_info(2, i)){
                max = obstacle_info(2, i);
              }
              if(min > obstacle_info(2, i)){
                min = obstacle_info(2, i);
              }
            }
          }
          cluster_divide_info(4, j) = max * min * M_PI;
        }
        break;
      }
      //確認
      ROS_INFO("3-----");
      ROS_INFO("last_num:%d", obstacle_num);
      ROS_INFO("------");

    }
  }

  void likelihood(){
    int i, j, count; 
    float x_ave_scalar;

    //cov
    for(j = 0; j < k; j++){
      count = 0;
      cov = Matrix2f::Zero();
      for(i = 0; i < obstacle_num; i++){
        if(cluster_info(2, select_cluster) == obstacle_info(4, i) && cluster_divide_info(2, j) == obstacle_info(3, i)){
          cov(0, 0) += pow(cluster_divide_info(0, j) - obstacle_info(0, i), 2);
          cov(1, 1) += pow(cluster_divide_info(1, j) - obstacle_info(1, i), 2);
          cov(0, 1) += (cluster_divide_info(0, j) - obstacle_info(0, i)) * (cluster_divide_info(1, j) - obstacle_info(1, i));
          count += 1;
        }
      }
      cov(0, 0) = cov(0, 0) / count;
      cov(1, 1) = cov(1, 1) / count;
      cov(0, 1) = cov(0, 1) / count;
      cov(1, 0) = cov(0, 1);

      cov_det[j] = cov.determinant();
      cluster_divide_info(7, j) = cov.determinant();

      //確認
      ROS_INFO("cov_det:%f", cov.determinant());

      log_f = VectorXf::Zero(cluster_divide_info(3, j));
      x = Vector2f::Zero();
      ave = Vector2f::Zero();
      x_ave = Vector2f::Zero();
      ave(0) = cluster_divide_info(0, j);
      ave(1) = cluster_divide_info(1, j);
      count = 0;
      for(i = 0; i < obstacle_num; i++){
        if(cluster_info(2, select_cluster) == obstacle_info(4, i) && cluster_divide_info(2, j) == obstacle_info(3, i)){
          x(0) = obstacle_info(0, i);
          x(1) = obstacle_info(1, i);
          x_ave = x - ave;
          x_ave_scalar = x_ave.transpose() * cov.inverse() * x_ave;
          log_f(count) = (float)log((float)exp(- x_ave_scalar / 2) / (float)sqrt((float)pow(2 * M_PI, 2) * cov.determinant()));
          count += 1;
        }
      }

      cluster_divide_info(5, j) = log_f.sum();

      //確認
      ROS_INFO("log_likelihood:%f, x_ave_scalar:%f", cluster_divide_info(5, j), x_ave_scalar);
    }
  }

  void bic(){
    float beta, alpha;
    int dimension = 2, q;
    Vector2f cluster_dif;
    cluster_dif = Vector2f::Zero();
    q = dimension * (dimension + 3) / 2;

    bic1 = -2 * cluster_info(5, select_cluster) + q * (float)log(cluster_info(3, select_cluster));

    cluster_dif(0) = cluster_divide_info(0, 0) - cluster_divide_info(0, 1);
    cluster_dif(1) = cluster_divide_info(1, 0) - cluster_divide_info(1, 1);
    beta = cluster_dif.norm() / (float)sqrt(cov_det[0] + cov_det[1]);
    alpha = 0.5 / (1 / 1 + (float)exp(-beta));
    bic2 = -2 * (cluster_info(3, select_cluster) * (float)log(alpha) + (cluster_divide_info(5, 0) + cluster_divide_info(5, 1)) / 2) + (2 * q) * (float)log(cluster_info(3, select_cluster));

    ROS_INFO("-------------bic1:%f--------------", bic1);
    ROS_INFO("beta:%f, alpha:%f", beta, alpha);
    ROS_INFO("-------------bic2:%f--------------", bic2);
  }

  void comparison(){
    int i, c, content, count = 0, new_c_num = 2, check;
    MatrixXf cluster_stack;
    cluster_stack = MatrixXf::Zero(8, cluster_num);

    //分割しない
    if(bic1 <= bic2 || no_belong_cluster == true){
      //check
      cluster_info(6, select_cluster) = 1;
      ROS_INFO("check! clu_x:%f, clu_y:%f", cluster_info(0, select_cluster), cluster_info(1, select_cluster));
    //分割する
    }else{
      //cluster_infoをstack
      for(c = 0; c < cluster_num; c++){
        check = false;
        if(cluster_info(2, c) != select_cluster){
          for(content = 0; content < 8; content++){
            cluster_stack(content, count) = cluster_info(content, c);
          }
          count += 1;
        }
        for(i = 0; i < obstacle_num; i++){
          if(obstacle_info(4, i) != select_cluster && obstacle_info(4, i) == c){
            obstacle_info(3, i) = new_c_num;
            check = true;
          }
        }
        if(check == true){
          new_c_num += 1;
        }
      }

      cluster_num += 1;

     //cluster_info(x, y, number, num_point_group, size, log_likelihood, check)
     cluster_info = MatrixXf::Zero(8, cluster_num);

      for(c = 0; c < cluster_num; c++){
        for(content = 0; content < 8; content++){
          if(c < 2){
            cluster_info(content, c) = cluster_divide_info(content, c);
          }else{
            cluster_info(content, c) = cluster_stack(content, c-2);
          }
          cluster_info(2, c) = c;

          ROS_INFO("content:%d, c:%d, %f", content, c, cluster_info(content, c));
        }
      }
      for(i = 0; i < obstacle_num; i++){
        obstacle_info(4, i) = obstacle_info(3, i);
      }
    }
  }

  void annexation(){///使ってない
    float bic_a, bic_b, beta, alpha, cluster_points_num[cluster_num], tmp;
    int dimension = 2, q, check, pass[cluster_num];
    Vector2f cluster_dif;
    cluster_dif = Vector2f::Zero();
    q = dimension * (dimension + 3) / 2;
    for(int i = 0; i < cluster_num; i++){
      ROS_INFO("---x:%f, y:%f, number:%f, num_point_group:%f, size:%f, log_likelihood:%f, check:%f", cluster_info(0, i), cluster_info(1, i), cluster_info(2, i), cluster_info(3, i), cluster_info(4, i), cluster_info(5, i), cluster_info(6, i));
      cluster_points_num[i] = cluster_info(3, i);
    }
    //インサーションソート
    for(int i = 1; i < cluster_num; i++){
      tmp = cluster_points_num[i];
      if(cluster_points_num[i - 1] > tmp){
        int j = i;
        do{
          cluster_points_num[i] = cluster_points_num[i - 1];
          j--;
        }while(j > 0 && cluster_points_num[j - 1] > tmp);
        cluster_points_num[j] = tmp;
      }
    }

    for(int i = 0; i < cluster_num; i++){
      pass[i] = -1;
    }

    int m = 0, o = 0;
    for(int i = 0; i < cluster_num - 1; i++){
      for(int j = i + 1; j < cluster_num; j++){
        for(int p = 0; p < cluster_num; p++){
          if(pass[p] == i || pass[p] == j){
            check = true;
          }
        }
        if(check == true){
          break;

        }else{
          for(int l = 0; l < cluster_num; l++){
            for(int n = 0; n < cluster_num; n++){
              if(cluster_points_num[i] == cluster_info(3, l) && cluster_points_num[j] == cluster_info(3, n)){
                bic_a = -2 * cluster_info(5, l) + q * (float)log(cluster_info(3, l));

                cluster_dif(0) = cluster_info(0, l) - cluster_info(0, n);
                cluster_dif(1) = cluster_info(1, l) - cluster_info(1, n);
                beta = cluster_dif.norm() / (float)sqrt(cluster_info(7, l) + cluster_info(7, n));
                alpha = 0.5 / (1 / 1 + (float)exp(-beta));
                bic_b = -2 * (cluster_info(3, l) * (float)log(alpha) + (cluster_info(5, l) + cluster_info(5, n)) / 2) + (2 * q) * (float)log(cluster_info(3, l));

                if(bic_a > bic_b){
                  ROS_WARN("hogehoge");///////
                  cluster_annexation_x[n] = (cluster_info(0, l) + cluster_info(0, n)) / 2;
                  cluster_annexation_y[n] = (cluster_info(1, l) + cluster_info(1, n)) / 2;
                  cluster_annexation_num[n] = cluster_info(2, n);
                  cluster_annexation_points[n] = cluster_info(3, l) + cluster_info(3, n);
                  cluster_annexation_del_num[n] = cluster_info(2, l);
                  m += 1;
                  pass[o] = i;
                  o += 1;
                  pass[o] = j;
                  o += 1;
                  del_num += 1;
                }
              }
            }
          }
        }

      }
    }
  }

  void run(){
    ros::Rate loop_rate(10);
    int kaisuu = 0, i, j;
    while(ros::ok()){
      if(callback == false || obstacle_num == 0){
      }else{
        while(true){
          if(check() == true){
            ROS_INFO("finish!!");
            break;
          }else{
            kmeans();
            likelihood();
            bic();
            comparison();
          }
        }
        //確認
        ROS_INFO("cluster_num:%d, kaisuu%d", cluster_num, kaisuu);

        cluster_annexation_x.clear();
        cluster_annexation_y.clear();
        cluster_annexation_points.clear();
        cluster_annexation_num.clear();
        cluster_annexation_del_num.clear();
        cluster_annexation_x.resize(cluster_num);
        cluster_annexation_y.resize(cluster_num);
        cluster_annexation_points.resize(cluster_num);
        cluster_annexation_num.resize(cluster_num);
        cluster_annexation_del_num.resize(cluster_num);
        del_num = 0;
        for(int i = 0; i < cluster_num; i++){
          cluster_annexation_num[i] = -1;
          cluster_annexation_del_num[i] = -1;
        }
/*
        if(cluster_num != 1){
          annexation();
        }
*/
        int del = false, ann = false, l = 0;
        objects.points.clear();
        objects.points.resize(cluster_num - del_num);
        objects.channels.resize(1);
        objects.channels[0].name = "number";
        objects.channels[0].values.resize(cluster_num - del_num);
        for(i = 0; i < cluster_num; i++){
          for(j = 0; j < cluster_num; j++){
            if(cluster_info(2, i) == cluster_annexation_del_num[j]){
              del = true;
              break;
            }else if(cluster_info(2, i) == cluster_annexation_num[j]){
              ann = true;
              break;
            }
          }
          if(del == true){
            del = false;
            break;
          }else if(ann == true){
            ann = false;
            objects.points[l].x = cluster_annexation_x[j];
            objects.points[l].y = cluster_annexation_y[j];
            objects.channels[0].values[l] = l;
            l += 1;
          }else{
            objects.points[l].x = cluster_info(0, i);
            objects.points[l].y = cluster_info(1, i);
            objects.channels[0].values[l] = l;
            l += 1;
          }
        }
        objects_pub.publish(objects);

        clustering.channels[0].name = "number";
        clustering.channels[0].values.resize(obstacle_num);
        for(i = 0; i < obstacle_num; i++){
          clustering.channels[0].values[i] = obstacle_info(4, i);
        }
        kaisuu += 1;
        obstacle_clustering_pub.publish(clustering);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber dynamic_obstacle_sub;
  ros::Publisher obstacle_clustering_pub;
  ros::Publisher objects_pub;

  sensor_msgs::PointCloud clustering;
  sensor_msgs::PointCloud objects;

  MatrixXf obstacle_info;
  MatrixXf cluster_info;
  MatrixXf cluster_divide_info;
  Matrix2f cov;

  Vector2f x;
  Vector2f ave;
  Vector2f x_ave;
  VectorXf log_f;

  vector<float> cluster_annexation_x, cluster_annexation_y, cluster_annexation_points, cluster_annexation_num, cluster_annexation_del_num;
  int obstacle_num, cluster_num, select_cluster, no_belong_cluster, callback, del_num;
  float cov_det[2], bic1, bic2;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "xmeans");
  Xmeans dynamic_obstacle;
  dynamic_obstacle.run();
  return 0;
}
