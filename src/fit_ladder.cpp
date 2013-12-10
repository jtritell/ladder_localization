#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <interactive_markers/interactive_marker_server.h>
#include <string.h>
#include <stdlib.h>
#include <map>

#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ladder_shaper/LadderState.h>

using namespace visualization_msgs;
using namespace std;

void makeModelMarker(std::string mesh, geometry_msgs::Pose pose, geometry_msgs::Vector3& scale);
void makeLengthMarker(geometry_msgs::Pose pos);
void changeRail(geometry_msgs::Pose pose);
void makeRungMarker();

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
const std::string directory = "package://ladder_shaper/ladder_meshes/";
//"file:///home/jordan/Downloads/ladder_meshes/";
geometry_msgs::Pose origin, right_origin;
geometry_msgs::Pose top,toprail,stairrail, length_marker;

geometry_msgs::Vector3 toscale;// = geometry_msgs::Vector3( 1.0, 1.0, 1.0);
std::map <std::string, std::string> meshes;
std::map <std::string, geometry_msgs::Vector3> scales;

double legs_angle;// 0;
double legs_width;// .75;
double initial_legs_width;
double inverse_scale;
double stair_length;
double dx;
double incline;
double origcosincline;
double origsinincline;

double rung_spacing_x;
double rung_spacing_z;

int num_rungs;//number of rungs

tf::Vector3 reference;
tf::Quaternion rotation;

ros::Publisher state_pub;

void initVars(){
  stair_length = .9013;
  legs_angle = 0.01;
  legs_width = .75;
  inverse_scale = 1/legs_width;
  toscale.x=toscale.y=toscale.z=1;
  
  incline = atan(1.5/stair_length);
  origcosincline = cos(incline);
  origsinincline = sin(incline);
  rung_spacing_x = -0.15;
  rung_spacing_z = 0.25;

  origin.position.x=0;
  origin.position.y=0;
  origin.position.z=0;

  top.position.z=1.5;
  top.position.x=-stair_length;
  top.position.y=.375;

  toprail.position.z=2.25;
  toprail.position.x=-1.5;
  toprail.position.y=0.0;  

  stairrail.position.z=2.25;
  stairrail.position.x=-0.75;
  stairrail.position.y=0.75;  

  right_origin = origin;
  right_origin.position.y=.75;

  reference = tf::Vector3(0.0,0.0,0.0);
  rotation = tf::Quaternion(0.0, 0.0, 0.0, 1.0);

  meshes.clear();
  meshes["top_rail"] = "top_rail.stl";
  meshes["stair_rail"] = "stair_rail.stl";
  meshes["support_rail"] = "support_rail.stl";
  meshes["top_rung"] = "top_rung.stl";
  //  meshes["middle_rungs"] = "middle_rungs.stl";
  meshes["bottom_rung"] = "bottom_rung.stl";
  //  std::string rung_mesh = "all_rungs_6.stl";
  //  rung_mesh.append(boost::lexical_cast<string>(num_rungs));
  //  rung_mesh.append(".stl");
  //meshes["all_rungs"] = "bottom_rung.stl";

  scales["top_rail"] = toscale;
  scales["stair_rail"] = toscale;
  scales["support_rail"] = toscale;
  scales["top_rung"] = toscale;
  //  scales["middle_rungs"] = toscale;
  scales["bottom_rung"] = toscale;
  //scales["all_rungs"] = toscale;
}

void sendState(){
    ladder_shaper::LadderState l;
    InteractiveMarker int_marker;
    server->get("MoveAll",int_marker);
    double left_x = int_marker.pose.position.x;
    double left_y = int_marker.pose.position.y;
    server->get("RotateAll",int_marker);
    l.xpos = (int_marker.pose.position.x + left_x) / 2;
    l.ypos = (int_marker.pose.position.y + left_y) / 2;
    server->get("BottomRung",int_marker);
    l.spacing_first = int_marker.pose.position.z;

    server->get("ChangeHeight",int_marker);
    double topz = int_marker.pose.position.z;
    l.spacing = (int_marker.pose.position.z - l.spacing_first)/(num_rungs - 1);

    server->get("RailHeight",int_marker);
    l.walkway_rail_height = int_marker.pose.position.z;

    server->get("StairRailHeight",int_marker);
    l.rail_height = int_marker.pose.position.z - 1.27;
    l.inclination = abs(incline*180/M_PI);
    l.rungs = num_rungs;
    l.legs_width = legs_width;
    l.direction = legs_angle;
    state_pub.publish(l);
}

void exportXML(string filename="/home/jordan/test/xml"){
  InteractiveMarker int_marker;
  server->get("MoveAll",int_marker);
  double left_x = int_marker.pose.position.x;
  double left_y = int_marker.pose.position.y;
  server->get("RotateAll",int_marker);
  double pos_x = (int_marker.pose.position.x + left_x) / 2;
  double pos_y = (int_marker.pose.position.y + left_y) / 2;


  server->get("BottomRung",int_marker);
  double spacing_first = int_marker.pose.position.z;

  server->get("ChangeHeight",int_marker);
  double topz = int_marker.pose.position.z;
  double spacing = (int_marker.pose.position.z - spacing_first)/(num_rungs - 1);

  server->get("RailHeight",int_marker);
  double walkway_rail_height = int_marker.pose.position.z;
  
  server->get("StairRailHeight",int_marker);
  double railheight = int_marker.pose.position.z - 1.27;
  double inclination = abs(incline*180/M_PI);

  ofstream myfile;
  myfile.open(filename.c_str());
  myfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" <<endl
	 <<"<ladder>"<<endl
	 <<"<location pos_x = \""<<pos_x
	 <<"\" pos_y = \""<<pos_y<<"\"/>"<<endl
         <<"<direction angle=\""<<legs_angle<<"\">"<<endl
	 <<  "<shape type = \"ship_ladder\" spacing_first = \""
	 <<spacing_first<<"\" spacing = \""<<spacing
     <<"\" inclination = \""<<inclination
     <<"\" num = \""<<num_rungs<<"\" narrow_angle = \"0\" stringer_width=\"0.0254\" />"<<endl
	 <<"<rung type = \"rectangle\" width = \""<<legs_width
	 <<"\" height = \"0.04\" depth = \"0.1778\" />"<<endl
	 <<"  <rail railheight = \""<<railheight<<"\" walkway_rail_height=\""<<walkway_rail_height<<"\" rail_radius=\"0.0127\" rail_cen_1st_rung=\"0\"/>"<<endl
	 <<"<physics kfriction = \"0.25\" />"<<endl
	 <<"</ladder>"<<endl;
  myfile.close();
  cout<<"exported XML"<<endl;
}

void moveFrame(const ros::TimerEvent){
  static tf::TransformBroadcaster br;
  
  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(reference);
  t.setRotation(rotation);
  br.sendTransform(tf::StampedTransform(t,time, "leftFoot", "workstation_ladder"));
  /*
  tf::Transform t2;
  t2.setRotation(tf::Quaternion());
  t2.setOrigin(tf::Vector3());
  br.sendTransform(tf::StampedTransform(t2,time, "leftFoot", "leftFoot"));
  */
}

void moveReference(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::Point p = feedback->pose.position;
  reference = tf::Vector3(p.x,p.y,p.z);

  geometry_msgs::Pose pose = feedback->pose;
  geometry_msgs::Pose right = pose;
  InteractiveMarker int_marker;
  
  right.position.x = pose.position.x - sin(legs_angle)*legs_width;
  right.position.y = pose.position.y + cos(legs_angle)*legs_width;
  server->setPose("RotateAll", right);
  server->applyChanges();
  sendState();
}

void rotateReference(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::Pose leader_pose = feedback->pose;
  InteractiveMarker int_marker;
  server->get("MoveAll",int_marker);
  float diff_x = int_marker.pose.position.x - leader_pose.position.x;
  float diff_y = int_marker.pose.position.y - leader_pose.position.y;

  double flip_angle = (diff_x > 0) * (M_PI);

  float new_w = sqrt(pow(diff_x*inverse_scale,2) + pow(diff_y*inverse_scale,2));
  double angle = atan(diff_y/diff_x) - M_PI/2 + flip_angle;
  legs_width = sqrt(pow(diff_x,2) + pow(diff_y,2));
  legs_angle = angle;
  rotation = tf::createQuaternionFromRPY(0.0,0.0,angle);

  for (std::map<std::string,std::string>::iterator it=meshes.begin(); it!=meshes.end(); ++it){
    server->get(it->first,int_marker);
    geometry_msgs::Pose pose = int_marker.pose;
    geometry_msgs::Vector3 scaled = scales[it->first];
    scaled.y = new_w;
    scales[it->first] = scaled;
    makeModelMarker(it->first,pose,scaled);    
  }

  server->get("BottomRung",int_marker);
  int_marker.pose.position.y = legs_width/2;
  server->setPose("BottomRung",int_marker.pose);

  server->get("ChangeHeight",int_marker);
  int_marker.pose.position.y = legs_width/2;
  server->setPose("ChangeHeight",int_marker.pose);

  server->get("StairRailHeight",int_marker);
  int_marker.pose.position.y = legs_width;
  server->setPose("StairRailHeight",int_marker.pose);
  
  server->applyChanges();  
}

void changeRungs(geometry_msgs::Pose bottom,geometry_msgs::Pose top){

  geometry_msgs::Point tpos = top.position;
  bottom.position.x -= 0.1 * (scales["stair_rail"].x - 1);
  geometry_msgs::Point bpos = bottom.position;
  bpos.y -= legs_width/2;
  bottom.position.x -= rung_spacing_x;
  bottom.position.y -= legs_width/2;
  bottom.position.z -= rung_spacing_z;
  //  double dx = top.position.x - bottom.position.x;
  //  double dz = top.position.z - bottom.position.z;
  //  double hypotenuse = sqrt(pow(dx,2) + pow(dz,2));
  geometry_msgs::Vector3 scaled = scales["bottom_rung"];
  //  scaled.x = abs(hypotenuse / 1.75 * (cos(incline) / origcosincline));
  //  scaled.z = abs(hypotenuse / 1.75 * (sin(incline) / origsinincline));
  //  scales["all_rungs"] = scaled;
  //  cout<<"dx:"<<dx<<" "<<"dz:"<<dz<<" hypotenuse:"<<hypotenuse<<endl;
  //  cout<<"compared sins:"<<(sin(incline) / origsinincline)<<" cos's:"<<(cos(incline) / origcosincline)<<endl;
  makeModelMarker("bottom_rung",bottom,scaled);
  tf::Vector3 newloc(bpos.x,bpos.y,bpos.z);
  newloc -= tf::Vector3(rung_spacing_x,0,rung_spacing_z);
  tf::Vector3 difference(tpos.x-bpos.x,0,tpos.z-bpos.z);
  difference/=(num_rungs - 1);
  geometry_msgs::Vector3 diff_step;
  tf::vector3TFToMsg(difference,diff_step);
  for(int i=1;i<num_rungs - 1;i++){
    newloc+=difference;
    std::string rungname= "rung_";
    rungname.append(boost::lexical_cast<string>(i));
    geometry_msgs::Pose rung_pose;
    //    geometry_msgs::
    tf::pointTFToMsg(newloc,rung_pose.position);
    makeModelMarker(rungname,rung_pose,scaled);
  }
    //  server->setPose("all_rungs",bottom);
  server->applyChanges();
}

void moveRungs(  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  InteractiveMarker int_marker;
  server->get("ChangeHeight",int_marker);
  geometry_msgs::Pose top = int_marker.pose;
  geometry_msgs::Pose bottom = feedback->pose;
  changeRungs(bottom,top);
}
  
//void adjustRungs(geometry_msgs::Pose bottom, geometry_msgs::Pose top){

void changeLengthHeight (  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  server->applyChanges();

  geometry_msgs::Pose pose = feedback->pose;
  incline = atan(pose.position.z/pose.position.x);
  dx = pose.position.x + stair_length;
  pose.position.z-=1.5;
  InteractiveMarker int_marker;
  server->get("top_rail",int_marker);

  double xscale = -pose.position.x / stair_length;

  geometry_msgs::Pose new_pose = int_marker.pose;

  new_pose.position.z = pose.position.z;

  server->get("stair_rail",int_marker);
  new_pose=int_marker.pose;
  new_pose.position.z=0;
  geometry_msgs::Vector3 scaled = scales["stair_rail"];
  scaled.x = xscale;
  scales["stair_rail"]=scaled;
  makeModelMarker("stair_rail", new_pose, scaled);

  scaled = scales["support_rail"];
  scaled.z=((1.5 + pose.position.z)/1.5);
  scaled.x = xscale;
  scales["support_rail"]=scaled;
  makeModelMarker("support_rail",new_pose,scaled);

  new_pose=int_marker.pose;
  new_pose.position.x += dx;
  new_pose.position.z=0;
  scaled = scales["top_rung"];
  scaled.z=((1.5 + pose.position.z)/1.5);
  scales["top_rung"]=scaled;
  makeModelMarker("top_rung",new_pose,scaled);

  server->get("RailHeight",int_marker);
  double new_pose_z = int_marker.pose.position.z;
  new_pose.position.z = new_pose_z;
  new_pose.position.x = pose.position.x - 1;
  server->setPose("RailHeight", new_pose);
  //  server->setPose("BottomRung", new_pose);
  changeRail(new_pose);
  makeRungMarker();
  server->get("BottomRung",int_marker);
  changeRungs(int_marker.pose,feedback->pose);
}

void changeStairRail(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  pose.position.z-=2.25;
  server->get("stair_rail",int_marker);  
  geometry_msgs::Pose new_pose = int_marker.pose;
  new_pose.position.z=0;
  geometry_msgs::Vector3 scaled = scales["stair_rail"];
  scaled.z=((2.25 + pose.position.z)/2.25);
  scales["stair_rail"]=scaled;
  makeModelMarker("stair_rail",new_pose,scaled);
}

void changeRail(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  pose.position.z -= 2.25;
  server->get("ChangeHeight",int_marker);
  geometry_msgs::Vector3 scaled = scales["top_rail"];
  double tr_height = int_marker.pose.position.z;
  double dz = 2.25 + pose.position.z - int_marker.pose.position.z;
  server->get("top_rail",int_marker);  
  geometry_msgs::Pose new_pose = int_marker.pose;
  new_pose.position.x = dx;
  scaled = scales["top_rail"];
  scaled.z=(dz/0.75);
  scales["top_rail"]=scaled;
  new_pose.position.z = tr_height - 1.5*scaled.z;
  makeModelMarker("top_rail",new_pose,scaled);
  
  server->applyChanges();
}

void changeRailHeight(  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::Pose pose;
  pose = feedback->pose;
  //   pose.position.x += 1.5;
 //  pose.position.x -=stair_length;
  changeRail(pose);
}

void changeStairRailHeight(  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  InteractiveMarker int_marker;
  geometry_msgs::Pose pose;
  pose = feedback->pose;
  pose.position.z-=2.25;
  server->get("stair_rail",int_marker);  
  geometry_msgs::Pose new_pose = int_marker.pose;
  new_pose.position.z=0;
  geometry_msgs::Vector3 scaled = scales["stair_rail"];
  scaled.z=((2.25 + pose.position.z)/2.25);
  scales["stair_rail"]=scaled;
  makeModelMarker("stair_rail",new_pose,scaled);
  server->applyChanges();

}

Marker makeModel(std::string filename){
  Marker model_marker;
  model_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  model_marker.mesh_resource = filename;
  model_marker.color.r = 0.5;
  model_marker.color.g = 0.5;
  model_marker.color.b = 0.5;
  model_marker.color.a = 0.5;
  return model_marker;
}


void makeModelMarker(std::string mesh, geometry_msgs::Pose pose = origin, geometry_msgs::Vector3& scale = toscale){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "workstation_ladder";
  int_marker.scale = .5;
  int_marker.name = mesh;
  int_marker.pose = pose;
  InteractiveMarkerControl control;
  //control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";
  std::string filename = directory + meshes[mesh];
  Marker marker = makeModel(filename);
  //  geometry_msgs::pointTFToMsg(scale,marker.scale);
  marker.scale.x = scale.x;
  marker.scale.y = scale.y;//scale.y;
  marker.scale.z = scale.z;
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  //  server->setCallback(int_marker.name, &stateCallback);
  sendState();
}

void makeCenterMarker(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "leftFoot";
  int_marker.name = "MoveAll";
  int_marker.pose = pose;
  InteractiveMarkerControl box_control;
  box_control.name = "CenterMarker";
  box_control.always_visible = true;

  Marker marker;
  marker.type = Marker::SPHERE;
  marker.scale.x = marker.scale.y = marker.scale.z = .10;
  marker.color.r = 0.7;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  box_control.markers.push_back(marker);
  //  int_marker.controls.push_back(box_control);

  InteractiveMarkerControl rotate_control;

  box_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // add the control to the interactive marker
  box_control.orientation.w=1;
  box_control.orientation.x=0;
  box_control.orientation.y=1;
  box_control.orientation.z=0;
  box_control.name = "move_x";
  int_marker.controls.push_back(box_control);

  rotate_control.name = "move_y";
  rotate_control.orientation.w=1;
  rotate_control.orientation.x=0;
  rotate_control.orientation.y=0;
  rotate_control.orientation.z=1;
  
  int_marker.controls.push_back(rotate_control);
  /*
  rotate_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  rotate_control.name = "move_z";
  rotate_control.orientation.w=1;
  rotate_control.orientation.x=0;
  rotate_control.orientation.y=1;
  rotate_control.orientation.z=0;
  
  int_marker.controls.push_back(rotate_control);
*/
  rotate_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  rotate_control.orientation.w=1;
  rotate_control.orientation.x=0;
  rotate_control.orientation.y=1;
  rotate_control.orientation.z=0;

  server->insert(int_marker);

  server->setCallback(int_marker.name, &moveReference, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}

void makeSideMarker(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "leftFoot";
  int_marker.name = "RotateAll";
  int_marker.pose = pose;
  InteractiveMarkerControl box_control;
  box_control.name = "SideMarker";
  box_control.always_visible = true;

  Marker marker;
  marker.type = Marker::SPHERE;
  marker.scale.x = marker.scale.y = marker.scale.z = .10;
  marker.color.r = 0.5;
  marker.color.g = 0.7;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  box_control.markers.push_back(marker);
  //  int_marker.controls.push_back(box_control);

  box_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // add the control to the interactive marker
  box_control.orientation.w=1;
  box_control.orientation.x=0;
  box_control.orientation.y=1;
  box_control.orientation.z=0;
  box_control.name = "move_xy";
  int_marker.controls.push_back(box_control);

  // we want to use our special callback function
  server->insert(int_marker);
  //  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &rotateReference, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}

void makeHeightMarker(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "workstation_ladder";
  int_marker.name = "ChangeHeight";
  int_marker.pose = pose;
  int_marker.scale = 0.5;
  InteractiveMarkerControl move_control;
  
  move_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  
  // add the control to the interactive marker
  move_control.orientation.w=1;
  move_control.orientation.x=0;
  move_control.orientation.y=1;
  move_control.orientation.z=0;
  move_control.name = "move_z";
  int_marker.controls.push_back(move_control);

  move_control.orientation.w=1;
  move_control.orientation.x=1;
  move_control.orientation.y=0;
  move_control.orientation.z=0;
  move_control.name = "move_y";
  int_marker.controls.push_back(move_control);

  server->insert(int_marker);  
  server->setCallback(int_marker.name, &changeLengthHeight, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}

void makeRailMarker(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "workstation_ladder";
  int_marker.name = "RailHeight";
  int_marker.pose = pose;
  int_marker.scale = 0.5;

  InteractiveMarkerControl move_control;
  
  move_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  
  // add the control to the interactive marker
  move_control.orientation.w=1;
  move_control.orientation.x=0;
  move_control.orientation.y=1;
  move_control.orientation.z=0;
  move_control.name = "move_z";
  int_marker.controls.push_back(move_control);

  server->insert(int_marker);  
  server->setCallback(int_marker.name, &changeRailHeight, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );

}

void makeStairRailMarker(geometry_msgs::Pose pose){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "workstation_ladder";
  int_marker.name = "StairRailHeight";
  int_marker.pose = pose;
  int_marker.scale = 0.5;

  InteractiveMarkerControl move_control;
  
  move_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  
  // add the control to the interactive marker
  move_control.orientation.w=1;
  move_control.orientation.x=0;
  move_control.orientation.y=1;
  move_control.orientation.z=0;
  move_control.name = "move_z";
  int_marker.controls.push_back(move_control);

  server->insert(int_marker);  
  server->setCallback(int_marker.name, &changeStairRailHeight, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );

}

void makeRungMarker(){
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "workstation_ladder";
  int_marker.name = "BottomRung";
  int_marker.scale = 0.5;

  int_marker.pose.position.x = rung_spacing_x;
  int_marker.pose.position.z = rung_spacing_z;
  int_marker.pose.position.y = legs_width/2;

  //  cout<<int_marker.pose<<endl;

  InteractiveMarkerControl move_control;
  
  tf::Quaternion rotation = tf::createQuaternionFromRPY(0.0,abs(incline),0.0);
  //  cout<<incline<<endl;
  move_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  tf::quaternionTFToMsg(rotation,move_control.orientation);  
  move_control.name = "move_xz";
  int_marker.controls.push_back(move_control);
  
  InteractiveMarkerControl box_control;
  /*
  Marker marker;
  marker.type = Marker::SPHERE;
  marker.scale.x = marker.scale.y = marker.scale.z = .10;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.7;
  marker.color.a = 1.0;
  box_control.markers.push_back(marker);
  */
  int_marker.controls.push_back(box_control);

  server->insert(int_marker);  
  server->setCallback(int_marker.name, &moveRungs, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}

void changeNumRungs(int rungs){
  int i;
  for(i=1;i<rungs - 1;i++){
    std::string rungname= "rung_";
    rungname.append(boost::lexical_cast<string>(i));
    meshes[rungname] = "bottom_rung.stl";
}
  for(;i<num_rungs;i++){
    std::string rungname= "rung_";
    rungname.append(boost::lexical_cast<string>(i));
    meshes.erase(rungname);
    server->erase(rungname);
}
  //  meshes.clear();
  num_rungs = rungs;
  InteractiveMarker int_marker;      
  server->get("BottomRung",int_marker);
  geometry_msgs::Pose bpos = int_marker.pose;
  server->get("ChangeHeight",int_marker);
  changeRungs(bpos,int_marker.pose);
}

void resetLadder(int rungs){
  initVars();
  server.reset( new interactive_markers::InteractiveMarkerServer("ladder_marker","",false) );
  
  for (std::map<std::string,std::string>::iterator it=meshes.begin(); it!=meshes.end(); ++it){
    makeModelMarker(it->first);
  }
  
  makeCenterMarker(origin);
  makeSideMarker(right_origin);
  makeHeightMarker(top);
  makeRailMarker(toprail);
  makeStairRailMarker(stairrail);
  makeRungMarker();
  changeNumRungs(rungs);
  sendState();
}


void exportCallback(const std_msgs::String::ConstPtr& msg)
{
    exportXML();
}

void resetCallback(const std_msgs::Int32::ConstPtr& msg){
    resetLadder(msg->data);
}

void rungsCallback(const std_msgs::Int32::ConstPtr& msg){
    changeNumRungs(msg->data);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ladder_marker");
    ros::NodeHandle n;

    ros::Timer frame_timer = n.createTimer(ros::Duration(0.05), moveFrame);
    state_pub =n.advertise<ladder_shaper::LadderState>("ladder_state",1000,true);
    num_rungs = 6;
    resetLadder(num_rungs);
    ros::Subscriber sub = n.subscribe("/export",1000,exportCallback);
    ros::Subscriber sub2 = n.subscribe("/reset",1000,resetCallback);
    ros::Subscriber sub3 = n.subscribe("/rungs",1000,rungsCallback);
//    ros::Publisher pub = n.advertise
    ros::spin();
}
 
