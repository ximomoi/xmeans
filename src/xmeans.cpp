#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <time.h>
#include <sstream>
#include <complex>
#include <stdlib.h>
#define obstacle_max 720
#define cluster_num_ini 1
#define k 2

using namespace Eigen;

class Xmeans{
public:
  Xmeans(){
    scan_sub = nh.subscribe("/scan", 10, &Xmeans::scanCallBack, this);
    obstacle_cluster_pub = nh.advertise<sensor_msgs::PointCloud>("/obstacle_cluster", 1000);

    //objects_info(x, y, k_distance, cluster_divide_num, cluster_num)
    objects_info = MatrixXf::Zero(5, obstacle_max);
    //cluster_info(x, y, number, num_point_group, size, log_likelihood, check)
    cluster_info = MatrixXf::Zero(7, cluster_num_ini);
    //cluster_divide_info(x, y, number, num_point_group, size, log_likelihood, check_ini)
    cluster_divide_info = MatrixXf::Zero(7, k);
   
    laser = VectorXf::Zero(obstacle_max);

    cluster.points.resize(obstacle_max);

    obstacle_last_num = obstacle_max;

    cluster_num = cluster_num_ini;

    ros::Rate loop_rate(10);
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
    int h = 0, i, x_max = -30, y_max = 0;
    float deg = 0, rad, add_coordinate_x = 0, add_coordinate_y = 0;
    ROS_INFO("-------------hogehoge---------------");
    for(i = 0; i < obstacle_max; i++){
      laser(i) = scan->ranges[i];
      if(laser(i) > 29){
        obstacle_last_num += -1;
      }else{
        rad = deg * M_PI / 180;
        objects_info(0, h) = scan->ranges[i] * cos(rad);
        objects_info(1, h) = scan->ranges[i] * sin(rad);
        add_coordinate_x += objects_info(0, h);
        add_coordinate_y += objects_info(1, h);
        if(x_max < objects_info(0, h)){
          x_max = objects_info(0, h);
        }
        if(y_max < objects_info(1, h)){
          y_max = objects_info(1, h);
        }
        h += 1;
      }
      deg += 0.25;
    }
    cluster_info(0, 0) = add_coordinate_x / obstacle_last_num;
    cluster_info(1, 0) = add_coordinate_y / obstacle_last_num;
    cluster_info(3, 0) = obstacle_last_num;
    cluster_info(4, 0) = (x_max - cluster_info(0, 0)) * (y_max - cluster_info(1, 0)) * M_PI;
 
    float x_ave_scalar;

    //cov
    cov = Matrix2f::Zero();
    for(h = 0; h < obstacle_last_num; h++){
      cov(0, 0) += pow(cluster_info(0, 0) - objects_info(0, h), 2);
      cov(1, 1) += pow(cluster_info(1, 0) - objects_info(1, h), 2);
      cov(0, 1) += (cluster_info(0, 0) - objects_info(0, h)) * (cluster_info(1, 0) - objects_info(1, h));
    }
    cov(0, 0) = cov(0, 0) / obstacle_last_num;
    cov(1, 1) = cov(1, 1) / obstacle_last_num;
    cov(0, 1) = cov(0, 1) / obstacle_last_num;
    cov(1, 0) = cov(0, 1);

    //確認
    ROS_INFO("cov_det:%f", cov.determinant());

    log_f = VectorXf::Zero(cluster_info(3, 0));
    x = Vector2f::Zero();
    ave = Vector2f::Zero();
    x_ave = Vector2f::Zero();
    ave(0) = cluster_info(0, 0);
    ave(1) = cluster_info(1, 0);
    for(h = 0; h < obstacle_last_num; h++){
        x(0) = objects_info(0, h);
        x(1) = objects_info(1, h);
        x_ave = x - ave;
        x_ave_scalar = x_ave.transpose() * cov.inverse() * x_ave;
        log_f(h) = (float)log((float)exp(- x_ave_scalar / 2) / (float)sqrt((float)pow(2 * M_PI, 2) * cov.determinant()));
    }

    cluster_info(5, 0) = log_f.sum();

    //確認
    ROS_INFO("log_likelihood:%f, x_ave_scalar:%f", cluster_info(5, 0), x_ave_scalar);
  }

  void kmeans(){
    int h, j, count, c;
    float deg_random, rad_random, range_random, distance, cluster_old_x[k], cluster_old_y[k], x_add, y_add;

    //取得した障害物の座標を元にランダムクラスタを配置する
    for(c = 0; c < cluster_num; c++){
      if(cluster_info(6, c) == 0){
        select_cluster = c;
        break;
      }
    }
    srand((unsigned int)time(NULL));
    for(j = 0; j < k; j++){
      deg_random = (float)rand() / ((float)RAND_MAX + 1) * 360;
      rad_random = deg_random * M_PI / 180;
      range_random = (float)rand() / ((float)RAND_MAX + 1);
      cluster_divide_info(0, j) = range_random * cos(rad_random) + cluster_info(0, select_cluster);
      cluster_divide_info(1, j) = range_random * sin(rad_random) + cluster_info(1, select_cluster);
      cluster_divide_info(2, j) = j;

      ROS_INFO("rand_clus_x:%f, rand_clus_y:%f", cluster_divide_info(0, j), cluster_divide_info(1, j));
    }

    int syuusoku = 0;
    while(true){
      int decision = 0;
      //クラスタの割り当て
      for(h = 0; h < obstacle_last_num; h++){
        if(cluster_info(2, select_cluster) == objects_info(4, h)){
          for(j = 0; j < k; j++){
            distance = hypotf(objects_info(0, h) - cluster_divide_info(0, j), objects_info(1, h) - cluster_divide_info(1, j));
            if(j == 0){
              objects_info(2, h) = distance;
              objects_info(3, h) = j;
            }else if(objects_info(2, h) > distance){
              objects_info(2, h) = distance;
              objects_info(3, h) = j;
            }
          }
        }
      }
      //確認
      ROS_INFO("1-----");
      ROS_INFO("obje_rang180:%f, obje_rang360:%f, obje_rang540:%f", laser(180), laser(360), laser(540));
      ROS_INFO("range_rand:%f, deg_rand:%f", range_random, deg_random);
      ROS_INFO("------");

      //クラスタの移動
      for(j = 0; j < k; j++){
        cluster_old_x[j] = cluster_divide_info(0, j);
        cluster_old_y[j] = cluster_divide_info(1, j);

        x_add = 0;
        y_add = 0;
        count = 0;
        for(h = 0; h < obstacle_last_num; h++){
          if(cluster_info(2, select_cluster) == objects_info(4, h) && cluster_divide_info(2, j) == objects_info(3, h)){
            x_add += objects_info(0, h);
            y_add += objects_info(1, h);
            count += 1;
          }
        }
        cluster_divide_info(3, j) = count;

        if(count == 0){
          ROS_INFO("cluster No:%d does not move", j);
        }else{
          cluster_divide_info(0, j) = x_add / count;
          cluster_divide_info(1, j) = y_add / count;
        }
        //確認
        ROS_INFO("2-----");
        ROS_INFO("count:%d, k:%d", count, j);

        ROS_INFO("k:%d, clus_old_x:%f, clus_old_y:%f", j, cluster_old_x[j], cluster_old_y[j]);
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
        float x_max = -30, y_max = 0;
          for(h = 0; h < obstacle_last_num; h++){
            if(cluster_info(2, select_cluster) == objects_info(4, h) && cluster_divide_info(2, j) == objects_info(3, h)){
              if(x_max < objects_info(0, h)){
                x_max = objects_info(0, h);
              }
              if(y_max < objects_info(1, h)){
                y_max = objects_info(1, h);
              }
            }
          }
          cluster_divide_info(4, j) = (x_max - cluster_divide_info(0, j)) * (y_max - cluster_divide_info(1, j)) * M_PI;
        }
        break;
      }
      //確認
      ROS_INFO("3-----");
      ROS_INFO("kaisuu:%d, last_num:%d", syuusoku, obstacle_last_num);
      ROS_INFO("------");

      syuusoku += 1;

    }
    ROS_INFO("--------------finish");
  }

  void likelihood(){
    int i, j, h, count; 
    float x_ave_scalar;

    //cov
    for(j = 0; j < k; j++){
      count = 0;
      cov = Matrix2f::Zero();
      for(h = 0; h < obstacle_last_num; h++){
        if(cluster_info(2, select_cluster) == objects_info(4, h) && cluster_divide_info(2, j) == objects_info(3, h)){
          cov(0, 0) += pow(cluster_divide_info(0, j) - objects_info(0, h), 2);
          cov(1, 1) += pow(cluster_divide_info(1, j) - objects_info(1, h), 2);
          cov(0, 1) += (cluster_divide_info(0, j) - objects_info(0, h)) * (cluster_divide_info(1, j) - objects_info(1, h));
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
      i = 0;
      for(h = 0; h < obstacle_last_num; h++){
        if(cluster_info(2, select_cluster) == objects_info(4, h) && cluster_divide_info(2, j) == objects_info(3, h)){
          x(0) = objects_info(0, h);
          x(1) = objects_info(1, h);
          x_ave = x - ave;
          x_ave_scalar = x_ave.transpose() * cov.inverse() * x_ave;
          log_f(i) = (float)log((float)exp(- x_ave_scalar / 2) / (float)sqrt((float)pow(2 * M_PI, 2) * cov.determinant()));
          i += 1;
        }
      }

      cluster_divide_info(5, j) = log_f.sum();

      //確認
      ROS_INFO("log_likelihood:%f, x_ave_scalar:%f", cluster_divide_info(5, j), x_ave_scalar);
    }
  }

  void bic(){
    float beta, alpha, bic1, bic2;
    int dimension = 2, q;
    Vector2f cluster_dif;
    cluster_dif = Vector2f::Zero();
    q = dimension * (dimension + 3) / 2;

    bic1 = cluster_info(5, select_cluster) - q * (float)log(cluster_info(3, select_cluster)) / 2;

    cluster_dif(0) = cluster_divide_info(0, 0) - cluster_divide_info(0, 1);
    cluster_dif(1) = cluster_divide_info(1, 0) - cluster_divide_info(1, 1);
    beta = cluster_dif.norm() / (float)sqrt(cov_det[0] + cov_det[1]);
    alpha = 0.5 / (1 / 1 + (float)exp(-beta));
    bic2 = (cluster_info(3, select_cluster) * (float)log(alpha) + cluster_divide_info(5, 0) + cluster_divide_info(5, 1)) - (2 * q) * (float)log(cluster_info(3, select_cluster)) / 2;

    ROS_INFO("-------------bic1:%f--------------", bic1);
    ROS_INFO("beta:%f, alpha:%f", beta, alpha);
    ROS_INFO("-------------bic2:%f--------------", bic2);
  }

  void comparison(){
    int h, c, content, count = 0, new_c_num = 2;
    MatrixXf cluster_stack;
    cluster_stack = MatrixXf::Zero(7, cluster_num);

    ROS_INFO("clu_num:%d", cluster_num);

    if(bic1 > bic2){
      //check
      cluster_info(6, select_cluster) = 1;
      ROS_INFO("check! clu_x:%f, clu_y:%f", cluster_info(0, select_cluster), cluster_info(1, select_cluster));
    }else{
      //cluster_infoをstack
      for(c = 0; c < cluster_num; c++){
        if(cluster_info(2, c) != select_cluster){
          for(content = 0; content < 7; content++){
            cluster_stack(content, count) = cluster_info(content, c);
          }
          count += 1;
        }
        for(h = 0; h < obstacle_last_num; h++){
          if(objects_info(4, h) != select_cluster && objects_info(4, h) == c){
            objects_info(3, h) = new_c_num;
          }
        }
        new_c_num += 1;
      }

      cluster_num += 1;

      for(c = 0; c < cluster_num; c++){
        for(content = 0; content < 7; content++){
          if(c < 2){
            cluster_info(content, c) = cluster_divide_info(content, c);
          }else{
            cluster_info(content, c) = cluster_stack(content, c-2);
          }
          cluster_info(2, c) = c;
        }
      }
      for(h = 0; h < obstacle_last_num; h++){
        objects_info(4, h) = objects_info(3, h);
      }
    }
  }

  void run(){
    int i;
    float bif_bic, aft_bic, bic_max;
    while(ros::ok()){
      int error = 0;
      for(i = 0; i < obstacle_max; i++){
        if(laser(i) == 0){
          error += 1;
        }
      }
      if(error == obstacle_max){
        ROS_ERROR("Laser scan value is zero");
      }else if(obstacle_last_num == 0){
        ROS_WARN("There is no dynamic obstacle");
      }else{
        kmeans();
        likelihood();
        bic();
        comparison();
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber scan_sub;
  ros::Publisher obstacle_cluster_pub;

  sensor_msgs::PointCloud cluster;

  MatrixXf objects_info;
  MatrixXf cluster_info;
  MatrixXf cluster_divide_info;
  Matrix2f cov;

  VectorXf laser;
  Vector2f x;
  Vector2f ave;
  Vector2f x_ave;
  VectorXf log_f;

  int obstacle_last_num, cluster_num, select_cluster;
  float cov_det[2], bic1, bic2;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "xmeans");
  Xmeans dynamic_obstacle;
  dynamic_obstacle.run();

  return 0;
}
