#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <limits>
#include <algorithm>

static std::string actuator_state;
static std::vector<geometry_msgs::Point> extended_footprint;
static std::vector<geometry_msgs::Point> original_footprint;
static std::vector<geometry_msgs::Point> current_footprint;

// current footprint params
static float max_x;
static float min_x = std::numeric_limits<float>::infinity();
static float max_y;
static float min_y = std::numeric_limits<float>::infinity();

sensor_msgs::LaserScan scan;
std::vector<double> scan_ranges;

void load_footprints(const ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* extended_footprint,
                       std::vector<geometry_msgs::Point>* original_footprint);
void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
void footprint_filter(sensor_msgs::LaserScanPtr output);
void actuator_state_callback(const std_msgs::String& msg);
bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>* footprint);
std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(const XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                                            const std::string& full_param_name);
std::vector<geometry_msgs::Point> return_footprint(XmlRpc::XmlRpcValue footprint_xmlrpc);
std::vector<std::vector<float> > parseVVF(const std::string& input, std::string* error_return);
geometry_msgs::Polygon returnPolygonFootprint(std::vector<geometry_msgs::Point> in_footprint);

class ScanFilter
{
  public:
  explicit ScanFilter(ros::NodeHandle n_)
  {
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan> ("unmasked_scan", 100, &ScanFilter::scan_callback, this);
    masked_scan_publisher = n_.advertise<sensor_msgs::LaserScan> ("scan", 1);
    raw_footprint_publisher = n_.advertise<geometry_msgs::Polygon> ("raw_footprint", 1);
  }

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = msg->header;
    output->header.frame_id = msg->header.frame_id;
    output->header.stamp = msg->header.stamp;
    output->angle_min = msg->angle_min;
    output->angle_max = msg->angle_max;
    output->angle_increment = msg->angle_increment;
    output->time_increment = msg->time_increment;
    output->scan_time = msg->scan_time;
    output->range_min = msg->range_min;
    output->range_max = msg->range_max;
    output->ranges = msg->ranges;

    footprint_filter(output);

    masked_scan_publisher.publish(output);

    geometry_msgs::Polygon raw_footprint = returnPolygonFootprint(current_footprint);
    raw_footprint_publisher.publish(raw_footprint);
    ROS_INFO("publshing a raw footprint");
  }

  private:
  ros::Subscriber scan_sub_;
  ros::Publisher masked_scan_publisher;
  ros::Publisher raw_footprint_publisher;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_filter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  load_footprints(private_nh, &extended_footprint, &original_footprint);

  ros::Subscriber actuator_state_sub_ = nh.subscribe("actuator_status", 1, actuator_state_callback);

  ScanFilter Filter(nh);

  ros::spin();

  return 0;
}




void load_footprints(const ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* extended_footprint,
                      std::vector<geometry_msgs::Point>* original_footprint)
{
  XmlRpc::XmlRpcValue footprint_xmlrpc1, footprint_xmlrpc2;
  nh.getParam("extended_footprint", footprint_xmlrpc1);
  nh.getParam("footprint", footprint_xmlrpc2);

  *extended_footprint = return_footprint(footprint_xmlrpc1);
  *original_footprint = return_footprint(footprint_xmlrpc2);
}

geometry_msgs::Polygon returnPolygonFootprint(std::vector<geometry_msgs::Point> in_footprint)
{
  geometry_msgs::Polygon raw_footprint;
  geometry_msgs::Point32 pt;

  for (int i = 0; i < in_footprint.size(); ++i)
  {
    geometry_msgs::Point point = in_footprint.at(i);
    pt.x = static_cast<float>(point.x);
    pt.y = static_cast<double>(point.y);

    raw_footprint.points.push_back(pt);
  }
  return raw_footprint;
}

void footprint_filter(sensor_msgs::LaserScanPtr output)
{
  size_t size = output->ranges.size();
  for (int i = 0; i < size; i++)
  {
    float angle = output->angle_min + output->angle_increment * i;
    float x = output->ranges[i] * cos(angle);
    float y = output->ranges[i] * sin(angle);

    // this if statement assumes footprint being square.
    // modify the condition for other footprint shapes
    if (min_x <= x && x <= max_x && min_y <= y && y <= max_y)
    {
      output->ranges[i] = std::numeric_limits<float>::infinity();
    }
  }
}


void actuator_state_callback(const std_msgs::String& msg)
{
  actuator_state = msg.data;

  if (actuator_state == "LOW")
  {
    current_footprint = original_footprint;
  }
  else if (actuator_state == "HIGH")
  {
    current_footprint = extended_footprint;
  }
  else
  {
    ROS_WARN("topic /actuator_status not being published or received. substituting original footprint...");
    current_footprint = original_footprint;
  }

  for (int i = 0; i < current_footprint.size(); i++)
  {
    min_x = std::min(min_x, static_cast<float>(current_footprint[i].x));
    min_y = std::min(min_y, static_cast<float>(current_footprint[i].y));
    max_x = std::max(max_x, static_cast<float>(current_footprint[i].x));
    max_y = std::max(max_y, static_cast<float>(current_footprint[i].y));
  }
  ROS_INFO("min_x: %f, max_x: %f, min_y: %f, max_y: %f", min_x, max_x, min_y, max_y);
}

bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>* footprint)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, &error);

  footprint->reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[ i ].size() == 2)
    {
      geometry_msgs::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      footprint->push_back(point);
    }
    else
    {
      ROS_ERROR("Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 static_cast<int>(vvf[ i ].size()));
      return false;
    }
  }

  return true;
}


std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(const XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name)
{
  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i)
  {
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    pt.x = static_cast<double>(point[ 0 ]);
    pt.y = static_cast<double>(point[ 1 ]);

    footprint.push_back(pt);
  }
  return footprint;
}

std::vector<geometry_msgs::Point> return_footprint(XmlRpc::XmlRpcValue footprint_xmlrpc)
{
  std::string full_param_name;
  std::vector<geometry_msgs::Point> points;

  if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
      footprint_xmlrpc != "" && footprint_xmlrpc != "[]")
  {
    if (makeFootprintFromString(std::string(footprint_xmlrpc), &points))
    {
      return points;
    }
  }
  else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    points = makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
    return points;
  }
  else
  {
    return points;
  }
}


std::vector<std::vector<float> > parseVVF(const std::string& input, std::string* error_return)
{
  std::vector<std::vector<float> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
    case EOF:
      break;
    case '[':
      depth++;
      if (depth > 2)
      {
        *error_return = "Array depth greater than 2";
        return result;
      }
      input_ss.get();
      current_vector.clear();
      break;
    case ']':
      depth--;
      if (depth < 0)
      {
        *error_return = "More close ] than open [";
        return result;
      }
      input_ss.get();
      if (depth == 1)
      {
        result.push_back(current_vector);
      }
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default:  // All other characters should be part of the numbers.
      if (depth != 2)
      {
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
        std::string temp = err_ss.str();
        *error_return = temp;
        return result;
      }
      float value;
      input_ss >> value;
      if (!!input_ss)
      {
        current_vector.push_back(value);
      }
      break;
    }
  }

  if (depth != 0)
  {
    *error_return = "Unterminated vector string.";
  }
  else
  {
    *error_return = "";
  }

  return result;
}
