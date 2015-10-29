#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <time.h>
#include <sstream>
#include <complex>
#include <stdlib.h>
#define cluster_num_ini 1
#define k 2

using namespace Eigen;

class Xmeans{
public:
  Xmeans(){
    dynamic_obstacle_sub = nh.subscribe("dynamic_obstacle", 100, &Xmeans::dynamic_obstacleCallBack, this);
    obstacle_clustering_pub = nh.advertise<sensor_msgs::PointCloud>("obstacle_clustering", 1000);

    //cluster_divide_info(x, y, number, num_point_group, size, log_likelihood, check_ini)
    cluster_divide_info = MatrixXf::Zero(7, k);

    callback = false;
  }

  void dynamic_obstacleCallBack(const sensor_msgs::PointCloud::ConstPtr& msg){
    int i, x_max = -30, y_max = 0;
    float deg = 180, rad, add_coordinate_x = 0, add_coordinate_y = 0, x_ave_scalar, distance;
    callback = true;
    obstacle_num = (int)msg->points.size();
    cluster_num = cluster_num_ini;
    //cluster_info(x, y, number, num_point_group, size, log_likelihood, check)
    cluster_info = MatrixXf::Zero(7, cluster_num);

    //objects_info(x, y, k_distance, cluster_divide_num, cluster_num)
    objects_info = MatrixXf::Zero(5, obstacle_num);

    ROS_INFO("-------------hogehoge---------------");
    for(i = 0; i < (int)msg->points.size(); i++){
      objects_info(0, i) = msg->points[i].x;
      objects_info(1, i) = msg->points[i].y;

      add_coordinate_x += objects_info(0, i);
      add_coordinate_y += objects_info(1, i);
      if(x_max < objects_info(0, i)){
        x_max = objects_info(0, i);
      }
      if(y_max < objects_info(1, i)){
        y_max = objects_info(1, i);
      }
    }
    cluster_info(0, 0) = add_coordinate_x / obstacle_num;
    cluster_info(1, 0) = add_coordinate_y / obstacle_num;
    cluster_info(3, 0) = obstacle_num;
    cluster_info(4, 0) = (x_max - cluster_info(0, 0)) * (y_max - cluster_info(1, 0)) * M_PI;
 
    //cov
    cov = Matrix2f::Zero();
    for(i = 0; i < obstacle_num; i++){
      cov(0, 0) += pow(cluster_info(0, 0) - objects_info(0, i), 2);
      cov(1, 1) += pow(cluster_info(1, 0) - objects_info(1, i), 2);
      cov(0, 1) += (cluster_info(0, 0) - objects_info(0, i)) * (cluster_info(1, 0) - objects_info(1, i));
    }
    cov(0, 0) = cov(0, 0) / obstacle_num;
    cov(1, 1) = cov(1, 1) / obstacle_num;
    cov(0, 1) = cov(0, 1) / obstacle_num;
    cov(1, 0) = cov(0, 1);

    //確認
    ROS_INFO("cov_det:%f", cov.determinant());

    log_f = VectorXf::Zero(cluster_info(3, 0));
    x = Vector2f::Zero();
    ave = Vector2f::Zero();
    x_ave = Vector2f::Zero();
    ave(0) = cluster_info(0, 0);
    ave(1) = cluster_info(1, 0);
    for(i = 0; i < obstacle_num; i++){
        x(0) = objects_info(0, i);
        x(1) = objects_info(1, i);
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
        if(cluster_info(2, select_cluster) == objects_info(4, i)){
          for(j = 0; j < k; j++){
            distance = hypotf(objects_info(0, i) - cluster_divide_info(0, j), objects_info(1, i) - cluster_divide_info(1, j));
            if(j == 0){
              objects_info(2, i) = distance;
              objects_info(3, i) = j;
            }else if(objects_info(2, i) > distance){
              objects_info(2, i) = distance;
              objects_info(3, i) = j;
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
          if(cluster_info(2, select_cluster) == objects_info(4, i) && cluster_divide_info(2, j) == objects_info(3, i)){
            x_add += objects_info(0, i);
            y_add += objects_info(1, i);
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
            if(cluster_info(2, select_cluster) == objects_info(4, i) && cluster_divide_info(2, j) == objects_info(3, i)){
              if(max < objects_info(2, i)){
                max = objects_info(2, i);
              }
              if(min > objects_info(2, i)){
                min = objects_info(2, i);
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
        if(cluster_info(2, select_cluster) == objects_info(4, i) && cluster_divide_info(2, j) == objects_info(3, i)){
          cov(0, 0) += pow(cluster_divide_info(0, j) - objects_info(0, i), 2);
          cov(1, 1) += pow(cluster_divide_info(1, j) - objects_info(1, i), 2);
          cov(0, 1) += (cluster_divide_info(0, j) - objects_info(0, i)) * (cluster_divide_info(1, j) - objects_info(1, i));
          count += 1;
        }
      }
      cov(0, 0) = cov(0, 0) / count;
      cov(1, 1) = cov(1, 1) / count;
      cov(0, 1) = cov(0, 1) / count;
      cov(1, 0) = cov(0, 1);

      cov_det[j] = cov.determinant();

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
        if(cluster_info(2, select_cluster) == objects_info(4, i) && cluster_divide_info(2, j) == objects_info(3, i)){
          x(0) = objects_info(0, i);
          x(1) = objects_info(1, i);
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
    cluster_stack = MatrixXf::Zero(7, cluster_num);

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
          for(content = 0; content < 7; content++){
            cluster_stack(content, count) = cluster_info(content, c);
          }
          count += 1;
        }
        for(i = 0; i < obstacle_num; i++){
          if(objects_info(4, i) != select_cluster && objects_info(4, i) == c){
            objects_info(3, i) = new_c_num;
            check = true;
          }
        }
        if(check == true){
          new_c_num += 1;
        }
      }

      cluster_num += 1;

     //cluster_info(x, y, number, num_point_group, size, log_likelihood, check)
     cluster_info = MatrixXf::Zero(7, cluster_num);

      for(c = 0; c < cluster_num; c++){
        for(content = 0; content < 7; content++){
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
        objects_info(4, i) = objects_info(3, i);
      }
    }
  }

  void run(){
    int c, kaisuu = 0;
    sensor_msgs::PointCloud clustering;
    while(ros::ok()){
      if(callback == false){
        ROS_ERROR("dynamic_obsracle is not call");
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

        clustering.points.resize(cluster_num);
        clustering.channels.resize(1);
        clustering.channels[0].name = "intensity";
        clustering.channels[0].values.resize(cluster_num);
        for(c = 0; c < cluster_num; c++){
          ROS_INFO("x%f, y%f, count%f, num%f, size%f", cluster_info(0, c), cluster_info(1, c), cluster_info(2, c), cluster_info(3, c), cluster_info(4, c));
          clustering.channels[0].values[c] = c;
          clustering.points[c].x = cluster_info(0, c);
          clustering.points[c].y = cluster_info(1, c);
        }
        clustering.header.frame_id = "base_link";
        kaisuu += 1;
        obstacle_clustering_pub.publish(clustering);
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber dynamic_obstacle_sub;
  ros::Publisher obstacle_clustering_pub;

  sensor_msgs::PointCloud cluster;

  MatrixXf objects_info;
  MatrixXf cluster_info;
  MatrixXf cluster_divide_info;
  Matrix2f cov;

  Vector2f x;
  Vector2f ave;
  Vector2f x_ave;
  VectorXf log_f;

  int obstacle_num, cluster_num, select_cluster, no_belong_cluster, callback;
  float cov_det[2], bic1, bic2;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "xmeans");
  Xmeans dynamic_obstacle;
  dynamic_obstacle.run();
  return 0;
}
