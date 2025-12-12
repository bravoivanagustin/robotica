#include <vector>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <robmovil_msgs/msg/landmark_array.hpp>
#include <robmovil_msgs/msg/landmark.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "landmark_detector.h"

#define LANDMARK_DIAMETER 0.1 // metros (0.1 = 10cm)

robmovil_ekf::LandmarkDetector::LandmarkDetector() :
    Node("landmark_detector_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer), transform_received(false)
{
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>( "/scan", rclcpp::QoS(10), std::bind(&LandmarkDetector::on_laser_scan, this, std::placeholders::_1));
  landmark_pub = this->create_publisher<robmovil_msgs::msg::LandmarkArray>("/landmarks", rclcpp::QoS(10));
  pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("/landmarks_pointcloud", rclcpp::QoS(10));

  
  this->declare_parameter("robot_frame", std::string("base_link"));
  this->declare_parameter("publish_robot_frame", std::string("base_link"));
  this->declare_parameter("laser_frame", std::string("laser"));

  this->get_parameter("robot_frame", robot_frame);
  this->get_parameter("publish_robot_frame", publish_robot_frame);
  this->get_parameter("laser_frame", laser_frame);

  RCLCPP_INFO(this->get_logger(), "Publishing to frame  %s", publish_robot_frame.c_str());
}

void robmovil_ekf::LandmarkDetector::on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if(!update_laser_tf(msg->header.stamp)){
    RCLCPP_WARN(this->get_logger(), "%s -> %s transform not yet received, not publishing landmarks", laser_frame.c_str(), robot_frame.c_str());
    return;
  }

  /* COMPLETAR: Convertir range,bearing a puntos cartesianos x,y,0. (ojo: descarta puntos en el "infinito")
   * Descartando aquellas mediciones por fuera de los rangos validos */
  std::vector<tf2::Vector3> cartesian;
  
  float range_min = msg->range_min;
  float range_max = msg->range_max;
  
  float angle_min = msg->angle_min;
  float angle_increment = msg->angle_increment;

  for (int i = 0; i < msg->ranges.size(); i++)
  {
    /* Utilizar la informacion del mensaje para filtrar y convertir */
    float range = msg->ranges[i];
    
    if (!std::isfinite(range)) continue;
    if (range < range_min || range > range_max) continue;

    float angle = angle_min + static_cast<float>(i)*angle_increment;

    /* COMPLETAR: p debe definirse con informacion valida y 
     * en coordenadas cartesianas */
    tf2::Vector3 p(range*std::cos(angle), range*std::sin(angle), 0.0);
    
    /* convierto el punto en relacion al marco de referencia del laser al marco del robot */
    p = laser_transform * p;
    cartesian.push_back(p);
  }

  /* Mensaje del arreglo de landmarks detectados */
  robmovil_msgs::msg::LandmarkArray landmark_array;
  landmark_array.header.stamp = msg->header.stamp;
  landmark_array.header.frame_id = publish_robot_frame;
  
  /* VECTORES AUXILIARES: Pueden utilizar landmark_points para ir acumulando
   * mediciones cercanas */
  std::vector<tf2::Vector3> landmark_points;
  
  // centroides estimados de los postes en coordenadas cartesianas
  std::vector<tf2::Vector3> centroids;
  
  const double cluster_threshold = LANDMARK_DIAMETER * 1.2; // distancia maxima entre puntos para considerarlos del mismo landmark
  double radius = LANDMARK_DIAMETER / 2.0;

  for (int i = 0; i < cartesian.size(); i++)
  {
    
    /* COMPLETAR: Acumular, de manera secuencial, mediciones cercanas (distancia euclidea) */
    
    // ----------------------------

    const tf2::Vector3& p = cartesian[i];

    // Primer punto de un cluster
    if (landmark_points.empty()) {
        landmark_points.push_back(p);
        continue;
    }
    
    else {
      const tf2::Vector3& last = landmark_points.back();
      double dx = p.getX() - last.getX();
      double dy = p.getY() - last.getY();
      double dist = hypot(dx, dy);

      if (dist <= cluster_threshold) {
        // Pertenece al mismo landmark
        landmark_points.push_back(p);
        continue;
      }
    }

    // Necesito sacarme el landmark que esta arriba del robot
    
    const tf2::Vector3& last = landmark_points.back();
    double dx_filtrado = last.getX() - landmark_points[0].getX();
    double dy_filtrado = last.getY() - landmark_points[0].getY();
    double dist_filtrado = hypot(dx_filtrado, dy_filtrado);

    // Ultimo agregado, esto hace que no tome al propio robot como landmark  
    if (dist_filtrado < 0.001) {
      landmark_points.clear();
      landmark_points.push_back(p);
      continue;
    }

    /* Al terminarse las mediciones provenientes al landmark que se venia detectando,
     * se calcula la pose del landmark como el centroide de las mediciones */

    RCLCPP_INFO(this->get_logger(), "landmark con %zu puntos", landmark_points.size());
    
    /* COMPLETAR: calcular el centroide de los puntos acumulados */

    tf2::Vector3 l_min = landmark_points[0];
    double min_norm = l_min.length();

    for (auto &pt : landmark_points) {
      double n = pt.length();
      if (n < min_norm) {
        min_norm = n;
        l_min = pt;
      }
    }

    tf2::Vector3 dir = l_min.normalized();

    tf2::Vector3 centroid = l_min + dir * radius;

    RCLCPP_INFO(this->get_logger(), "landmark detectado (cartesianas): %f %f %f", centroid.getX(), centroid.getY(), centroid.getZ());
    centroids.push_back(centroid);

    /* Convertir el centroide a coordenadas polares, construyendo el mensaje requerido */
    robmovil_msgs::msg::Landmark landmark;

    float r = hypot(centroid.getX(), centroid.getY()); // distancia desde el robot al centroide
    landmark.range = r;

    float a = atan2(centroid.getY(), centroid.getX()); // angulo de la recta que conecta al robot con el centroide
    landmark.bearing = a;

    /* Fin Completar */

    /* se agrega el landmark en coordenadas polares */
    landmark_array.landmarks.push_back(landmark);
    RCLCPP_INFO(this->get_logger(), "landmark detectado (polares):  %u: %f %f ", i, landmark.range, landmark.bearing);

    /* empiezo a procesar un nuevo landmark */
    landmark_points.clear();
  }

  /* Publicamos el mensaje de los landmarks encontrados */
  if (!landmark_array.landmarks.empty()){
    landmark_pub->publish(landmark_array);
    publish_pointcloud(landmark_array.header, centroids);
  }
}

bool robmovil_ekf::LandmarkDetector::update_laser_tf(const rclcpp::Time& required_time)
{
  try
  {
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer.lookupTransform(robot_frame, laser_frame, required_time, tf2::durationFromSec(0.1));
    tf2::fromMsg(transformStamped.transform, laser_transform);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "No se pudo transformar %s a %s: %s",
              robot_frame.c_str(), laser_frame.c_str(), ex.what());
    return false;
  }
}

void robmovil_ekf::LandmarkDetector::publish_pointcloud(const std_msgs::msg::Header& header, const std::vector<tf2::Vector3>& landmark_positions)
{
  sensor_msgs::msg::PointCloud pointcloud;
  pointcloud.header.stamp = header.stamp;
  pointcloud.header.frame_id = header.frame_id;

  for (int i = 0; i < landmark_positions.size(); i++)
  {
    geometry_msgs::msg::Point32 point;
    point.x = landmark_positions[i].getX();
    point.y = landmark_positions[i].getY();
    point.z = landmark_positions[i].getZ();
    pointcloud.points.push_back(point);
  }
  pointcloud_pub->publish(pointcloud);
}
