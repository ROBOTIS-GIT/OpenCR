/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */

/**
 * this code is compatible with d_open_manipulator_chain.ino
**/

// Multiple Window
ChildApplet child;

// Control Interface
import controlP5.*;

// Init serial
import processing.serial.*;

// Shape variables
PShape goal_link1_shape, goal_link2_shape, goal_link3_shape, goal_link4_shape, goal_link5_shape, goal_left_palm_shape, goal_right_palm_shape;
PShape ctrl_link1_shape, ctrl_link2_shape, ctrl_link3_shape, ctrl_link4_shape, ctrl_link5_shape, ctrl_left_palm_shape, ctrl_right_palm_shape;

// Model pose
float model_trans_x, model_trans_y, model_trans_z, model_scale_factor;

// World pose
float world_rot_x, world_rot_y;

// Serial variable
Serial opencr_port;

// Angle variable
float[] receive_joint_angle = new float[4];
float[] receive_gripper_pos = new float[2];

float[] ctrl_joint_angle = new float[4];
float[] ctrl_gripper_pos = new float[2];

/*******************************************************************************
* Setting window size
*******************************************************************************/
void settings()
{
  size(900, 900, OPENGL);
}

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  surface.setTitle("OpenManipulator Chain");
  child = new ChildApplet();

  initShape();
  initView();

  connectOpenCR(0); // It is depend on laptop enviroments.
}

/*******************************************************************************
* Draw (loop function)
*******************************************************************************/
void draw()
{
  setWindow();

  drawTitle();
  drawWorldFrame();

  drawManipulator();
}

/*******************************************************************************
* Connect OpenCR
*******************************************************************************/
void connectOpenCR(int port_num)
{
  printArray(Serial.list());

  String port_name = Serial.list()[port_num];
  opencr_port = new Serial(this, port_name, 57600);
  opencr_port.bufferUntil('\n');
}

/*******************************************************************************
* Serial Interrupt
*******************************************************************************/
void serialEvent(Serial opencr_port)
{
  String opencr_string = opencr_port.readStringUntil('\n');
  opencr_string = trim(opencr_string);

  String[] cmd = split(opencr_string, ',');

  if (cmd[0].equals("angle"))
  {
    for (int cmd_cnt = 1; cmd_cnt < cmd.length; cmd_cnt++)
    {
      receive_joint_angle[cmd_cnt-1] = float(cmd[cmd_cnt]);
      print("joint " + cmd_cnt + ": " + cmd[cmd_cnt] + "  ");
    }
    println("");
  }
  else if (cmd[0].equals("tool"))
  {
    float angle2pos = map(float(cmd[1]), 0.907, -1.13, 0.010*1000, 0.035 * 1000);
    receive_gripper_pos[0] = angle2pos;
    receive_gripper_pos[1] = ctrl_gripper_pos[0] * (-2);
    
    print("tool : " + cmd[1]);
    println("");
  }
  else
  {
    println("Error");
  }
}

/*******************************************************************************
* Init viewpoint and camera
*******************************************************************************/
void initView()
{
  float camera_y = height/2.0;
  float fov = 200/float(width) * PI/2;
  float camera_z = camera_y / tan(fov / 2.0);
  float aspect = float(width)/float(height);

  perspective(fov, aspect, camera_z/10.0, camera_z*10.0);

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0, height/2.0-500, height/2.0 * 4,
         width/2-100, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Get shape
*******************************************************************************/
void initShape()
{
  goal_link1_shape       = loadShape("meshes/chain_link1.obj");
  goal_link2_shape       = loadShape("meshes/chain_link2.obj");
  goal_link3_shape       = loadShape("meshes/chain_link3.obj");
  goal_link4_shape       = loadShape("meshes/chain_link4.obj");
  goal_link5_shape       = loadShape("meshes/chain_link5.obj");
  goal_left_palm_shape   = loadShape("meshes/chain_link_grip_l.obj");
  goal_right_palm_shape  = loadShape("meshes/chain_link_grip_r.obj");

  ctrl_link1_shape       = loadShape("meshes/chain_link1.obj");
  ctrl_link2_shape       = loadShape("meshes/chain_link2.obj");
  ctrl_link3_shape       = loadShape("meshes/chain_link3.obj");
  ctrl_link4_shape       = loadShape("meshes/chain_link4.obj");
  ctrl_link5_shape       = loadShape("meshes/chain_link5.obj");
  ctrl_left_palm_shape   = loadShape("meshes/chain_link_grip_l.obj");
  ctrl_right_palm_shape  = loadShape("meshes/chain_link_grip_r.obj");

  ctrl_link1_shape.setFill(color(200));
  ctrl_link2_shape.setFill(color(200));
  ctrl_link3_shape.setFill(color(200));
  ctrl_link4_shape.setFill(color(200));
  ctrl_link5_shape.setFill(color(200));
  ctrl_left_palm_shape.setFill(color(200));
  ctrl_right_palm_shape.setFill(color(200));

  setCtrlJointAngle(0, 0, 0, 0);
  gripperOff();

  receive_gripper_pos[0] = 0.030 * 1000;
  receive_gripper_pos[1] = receive_gripper_pos[0] * (-2);
}

/*******************************************************************************
* Set window characteristic
*******************************************************************************/
void setWindow()
{
  lights();
  smooth();
  background(30);

  translate(width/2, height/2, 0);

  rotateX(radians(90));
  rotateZ(radians(140));
}

/*******************************************************************************
* Draw sphere
*******************************************************************************/
void drawSphere(int x, int y, int z, int r, int g, int b, int size)
{
  pushMatrix();
  translate(x,y,z);
  stroke(r,g,b);
  sphere(size);
  popMatrix();
}

/*******************************************************************************
* Draw title
*******************************************************************************/
void drawTitle()
{
  pushMatrix();
  rotateX(radians(60));
  rotateZ(radians(180));
  textSize(60);
  fill(255,204,102);
  text("OpenManipulator Chain", -450,75,0);
  textSize(20);
  fill(102,255,255);
  text("Move manipulator 'Q,A','W,S','E,D'", -450,120,0);
  text("Initial view 'I'", -450,150,0);
  popMatrix();
}

/*******************************************************************************
* Draw manipulator
*******************************************************************************/
void drawManipulator()
{
  scale(1 + model_scale_factor);

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(0.0);
  rotateZ(0.0);
  shape(goal_link1_shape);
  drawLocalFrame();

  translate(0.0, 0.0, 0.036*1000);
  rotateZ(-receive_joint_angle[0]);
  shape(goal_link2_shape);
  drawLocalFrame(); 

  translate(0.0, 0.0, 0.040*1000);
  rotateY(receive_joint_angle[1]);
  shape(goal_link3_shape);
  drawLocalFrame();

  translate(0.024*1000, 0.0, 0.128*1000);
  rotateY(receive_joint_angle[2]);
  shape(goal_link4_shape);
  drawLocalFrame();

  translate(0.124*1000, 0.0, 0.0);
  rotateY(receive_joint_angle[3]);
  shape(goal_link5_shape);
  drawLocalFrame();

  translate(0.130*1000, 0.007*1000, 0);
  drawSphere(0, -7, 0, 100, 100, 100, 10);
  translate(0, receive_gripper_pos[0], 0);
  shape(goal_right_palm_shape);
  drawLocalFrame();

  translate(0, -0.014*1000, 0);
  translate(0, receive_gripper_pos[1], 0);
  shape(goal_left_palm_shape);
  drawLocalFrame();
  popMatrix();

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(0.0);
  rotateZ(0.0);
  shape(ctrl_link1_shape);
  drawLocalFrame();

  translate(0.0, 0.0, 0.036*1000);
  rotateZ(-ctrl_joint_angle[0]);
  shape(ctrl_link2_shape);
  drawLocalFrame(); 

  translate(0.0, 0.0, 0.040*1000);
  rotateY(ctrl_joint_angle[1]);
  shape(ctrl_link3_shape);
  drawLocalFrame();

  translate(0.024*1000, 0.0, 0.128*1000);
  rotateY(ctrl_joint_angle[2]);
  shape(ctrl_link4_shape);
  drawLocalFrame();

  translate(0.124*1000, 0.0, 0.0);
  rotateY(ctrl_joint_angle[3]);
  shape(ctrl_link5_shape);
  drawLocalFrame();

  translate(0.130*1000, 0.007*1000, 0);
  drawSphere(0, -7, 0, 100, 100, 100, 10);
  translate(0, ctrl_gripper_pos[0], 0);
  shape(ctrl_right_palm_shape);
  drawLocalFrame();

  translate(0, -0.014*1000, 0);
  translate(0, ctrl_gripper_pos[1], 0);
  shape(ctrl_left_palm_shape);
  drawLocalFrame();
  popMatrix();
}

/*******************************************************************************
* Draw world frame
*******************************************************************************/
void drawWorldFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0, 0, 200, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, -200, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 200);
}

/*******************************************************************************
* Draw local frame
*******************************************************************************/
void drawLocalFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 60, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, -60, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 60);
}

/*******************************************************************************
* Set joint angle
*******************************************************************************/
void setCtrlJointAngle(float angle1, float angle2, float angle3, float angle4)
{
  ctrl_joint_angle[0] = angle1;
  ctrl_joint_angle[1] = angle2;
  ctrl_joint_angle[2] = angle3;
  ctrl_joint_angle[3] = angle4;
}

/*******************************************************************************
* Gripper on
*******************************************************************************/
void gripperOn()
{
  ctrl_gripper_pos[0] = 0.020 * 1000;
  ctrl_gripper_pos[1] = ctrl_gripper_pos[0] * (-2);
}

/*******************************************************************************
* Gripper off
*******************************************************************************/
void gripperOff()
{
  ctrl_gripper_pos[0] = 0.030 * 1000;
  ctrl_gripper_pos[1] = ctrl_gripper_pos[0] * (-2);
}

/*******************************************************************************
* Gripper angle to position
*******************************************************************************/
void gripperAngle2Pos(float angle)
{
  float angle2pos = map(angle, 0.907, -1.13, 0.010*1000, 0.035 * 1000);

  ctrl_gripper_pos[0] = angle2pos;
  ctrl_gripper_pos[1] = ctrl_gripper_pos[0] * (-2);
}

/*******************************************************************************
* Mouse drag event
*******************************************************************************/
void mouseDragged()
{
  world_rot_x -= (mouseX - pmouseX) * 2.0;
  world_rot_y -= (mouseY - pmouseY) * 2.0;

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0 + world_rot_x, height/2.0-500 + world_rot_y, height/2.0 * 4,
         width/2-100, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Key press event
*******************************************************************************/
void keyPressed()
{
  if      (key == 'q') model_trans_x      -= 0.050 * 1000;
  else if (key == 'a') model_trans_x      += 0.050 * 1000;
  else if (key == 'w') model_trans_y      += 0.050 * 1000;
  else if (key == 's') model_trans_y      -= 0.050 * 1000;
  else if (key == 'e') model_trans_z      -= 0.050 * 1000;
  else if (key == 'd') model_trans_z      += 0.050 * 1000;
  else if (key == 'r') model_scale_factor += 0.5;
  else if (key == 'f') model_scale_factor -= 0.5;
  else if (key == 'i') 
  {
    model_trans_x = model_trans_y = model_scale_factor = world_rot_x = world_rot_y = 0.0;
    camera(width/2.0, height/2.0-500, height/2.0 * 4,
           width/2-100, height/2, 0,
           0, 1, 0);
  }
}

/*******************************************************************************
* Controller Window
*******************************************************************************/
class ChildApplet extends PApplet
{
  ControlP5 cp5;

  Textlabel headLabel;

  Knob joint1, joint2, joint3, joint4, gripper;

  float grip_angle;
  boolean onoff_flag = false;
  int motion_num = 0;

  public ChildApplet()
  {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings()
  {
    size(400, 600);
    smooth();
  }
  public void setup()
  {
    surface.setTitle("Control Interface");

    cp5 = new ControlP5(this);

/*******************************************************************************
* Init Tab
*******************************************************************************/
    cp5.addTab("Task Space Control")
       .setColorBackground(color(188, 188, 90))
       .setColorLabel(color(255))
       .setColorActive(color(0,128,0))
       ;

    cp5.addTab("Hand Teaching")
       .setColorBackground(color(100, 160, 0))
       .setColorLabel(color(255))
       .setColorActive(color(0,0,255))
       ;

    cp5.addTab("Motion")
       .setColorBackground(color(0, 160, 100))
       .setColorLabel(color(255))
       .setColorActive(color(0,0,255))
       ;

    cp5.getTab("default")
       .activateEvent(true)
       .setLabel("Joint Space Control")
       .setId(1)
       ;

    cp5.getTab("Task Space Control")
       .activateEvent(true)
       .setId(2)
       ;

    cp5.getTab("Hand Teaching")
       .activateEvent(true)
       .setId(3)
       ;

    cp5.getTab("Motion")
       .activateEvent(true)
       .setId(4)
       ;

/*******************************************************************************
* Init Joint Space Controller
*******************************************************************************/
    headLabel = cp5.addTextlabel("Label")
                   .setText("Controller for OpenManipulator Chain")
                   .setPosition(10,20)
                   .setColorValue(0xffffff00)
                   .setFont(createFont("arial",20))
                   ;

    cp5.addToggle("Controller_OnOff")
       .setPosition(0,50)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

    joint1 = cp5.addKnob("joint1")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(30,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint2 = cp5.addKnob("joint2")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(150,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint3 = cp5.addKnob("joint3")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(270,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint4 = cp5.addKnob("joint4")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(85,260)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    gripper = cp5.addKnob("gripper")
                .setRange(-1.13, 0.907)
                .setValue(0.0)
                .setPosition(210,260)
                .setRadius(50)
                .setDragDirection(Knob.HORIZONTAL)
                .setFont(createFont("arial",10))
                .setColorForeground(color(255))
                .setColorBackground(color(0, 160, 100))
                .setColorActive(color(255,255,0))
                ;

    cp5.addButton("Origin")
       .setValue(0)
       .setPosition(0,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Basic")
       .setValue(0)
       .setPosition(320,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Send_Joint_Angle")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Set_Gripper")
       .setValue(0)
       .setPosition(0,460)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Gripper_OnOff")
       .setPosition(0,520)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

/*******************************************************************************
* Init Task Space Controller
*******************************************************************************/
    cp5.addButton("Forward")
       .setValue(0)
       .setPosition(150,150)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Back")
       .setValue(0)
       .setPosition(150,350)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Left")
       .setValue(0)
       .setPosition(50,250)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Right")
       .setValue(0)
       .setPosition(250,250)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Set")
       .setValue(0)
       .setPosition(170,270)
       .setSize(60,60)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Up")
       .setValue(0)
       .setPosition(50,450)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

   cp5.addButton("Down")
      .setValue(0)
      .setPosition(250,450)
      .setSize(100,100)
      .setFont(createFont("arial",15))
      ;

   cp5.addToggle("Gripper_pos")
      .setPosition(165,480)
      .setSize(70,70)
      .setMode(Toggle.SWITCH)
      .setFont(createFont("arial",10))
      .setColorActive(color(196, 196, 196))
      .setColorBackground(color(255, 255, 153))
      ;

/*******************************************************************************
* Init Hand Teaching Controller
*******************************************************************************/
    cp5.addToggle("Torque_OnOff")
       .setPosition(0,130)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;
       
    cp5.addButton("Motion_Clear")
       .setValue(0)
       .setPosition(0,210)
       .setSize(200,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Make_Joint_Pose")
       .setValue(0)
       .setPosition(200,210)
       .setSize(200,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Make_Gripper_Pose")
       .setPosition(0,320)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

    cp5.addButton("Motion_Start")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,80)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Motion_Repeat")
       .setPosition(0,490)
       .setSize(400,80)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

/*******************************************************************************
* Init Motion
*******************************************************************************/
    cp5.addButton("Start")
       .setValue(0)
       .setPosition(0,200)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Stop")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

/*******************************************************************************
* Set Tap UI
*******************************************************************************/
    cp5.getController("Label").moveTo("global");
    cp5.getController("Controller_OnOff").moveTo("global");

    cp5.getController("Forward").moveTo("Task Space Control");
    cp5.getController("Back").moveTo("Task Space Control");
    cp5.getController("Left").moveTo("Task Space Control");
    cp5.getController("Right").moveTo("Task Space Control");
    cp5.getController("Set").moveTo("Task Space Control");
    cp5.getController("Up").moveTo("Task Space Control");
    cp5.getController("Down").moveTo("Task Space Control");
    cp5.getController("Gripper_pos").moveTo("Task Space Control");

    cp5.getController("Torque_OnOff").moveTo("Hand Teaching");
    cp5.getController("Motion_Clear").moveTo("Hand Teaching");
    cp5.getController("Make_Joint_Pose").moveTo("Hand Teaching");
    cp5.getController("Make_Gripper_Pose").moveTo("Hand Teaching");
    cp5.getController("Motion_Start").moveTo("Hand Teaching");
    cp5.getController("Motion_Repeat").moveTo("Hand Teaching");

    cp5.getController("Start").moveTo("Motion");
    cp5.getController("Stop").moveTo("Motion");
  }

  public void draw()
  {
    background(0);
  }

/*******************************************************************************
* Init Function of Joint Space Controller
*******************************************************************************/
  void Controller_OnOff(boolean flag)
  {
    onoff_flag = flag;
    if (onoff_flag)
    {
      joint1.setValue(ctrl_joint_angle[0]);
      joint2.setValue(ctrl_joint_angle[1]);
      joint3.setValue(ctrl_joint_angle[2]);
      joint4.setValue(ctrl_joint_angle[3]);

      opencr_port.write("opm"   + ',' +
                        "ready" + '\n');
      println("OpenManipulator Chain Ready!!!");
    }
    else
    {
      opencr_port.write("opm"  + ',' +
                        "end"  + '\n');
      println("OpenManipulator Chain End...");
    }
  }

  void joint1(float angle)
  {
    ctrl_joint_angle[0] = angle;
  }

  void joint2(float angle)
  {
    ctrl_joint_angle[1] = angle;
  }

  void joint3(float angle)
  {
    ctrl_joint_angle[2] = angle;
  }

  void joint4(float angle)
  {
    ctrl_joint_angle[3] = angle;
  }

  void gripper(float angle)
  {
    grip_angle = angle;
    gripperAngle2Pos(angle);
  }

  public void Origin(int theValue)
  {
    if (onoff_flag)
    {
      ctrl_joint_angle[0] = 0.0;
      ctrl_joint_angle[1] = 0.0;
      ctrl_joint_angle[2] = 0.0;
      ctrl_joint_angle[3] = 0.0;

      opencr_port.write("joint"            + ',' +
                        ctrl_joint_angle[0] + ',' +
                        ctrl_joint_angle[1] + ',' +
                        ctrl_joint_angle[2] + ',' +
                        ctrl_joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Basic(int theValue)
  {
    if (onoff_flag)
    {
      ctrl_joint_angle[0] = 0.0;
      ctrl_joint_angle[1] = -60.0  * PI/180.0;
      ctrl_joint_angle[2] = 20.0 * PI/180.0;
      ctrl_joint_angle[3] = 40.0 * PI/180.0;

      opencr_port.write("joint"        + ',' +
                        ctrl_joint_angle[0] + ',' +
                        ctrl_joint_angle[1] + ',' +
                        ctrl_joint_angle[2] + ',' +
                        ctrl_joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Send_Joint_Angle(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("joint"        + ',' +
                        ctrl_joint_angle[0] + ',' +
                        ctrl_joint_angle[1] + ',' +
                        ctrl_joint_angle[2] + ',' +
                        ctrl_joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Set_Gripper(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("gripper"  + ',' +
                        grip_angle + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Gripper_OnOff(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        // gripper.setValue(0.0);
        opencr_port.write("grip"  + ',' +
                          "on" + '\n');
      }
      else
      {
        // gripper.setValue(0.0);
        opencr_port.write("grip"  + ',' +
                          "off" + '\n');
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Task Space Controller
*******************************************************************************/
  public void Forward(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "forward" + '\n');
      println("Move Forward");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Back(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "back"    + '\n');
      println("Move Back");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Left(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "left"    + '\n');
      println("Move Left");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Right(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "right"   + '\n');
      println("Move Right");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Up(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "up"      + '\n');
      println("Move Up");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Down(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "down"    + '\n');
      println("Move Down");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Set(int theValue)
  {
    if (onoff_flag)
    {
      ctrl_joint_angle[0] = 0.0;
      ctrl_joint_angle[1] = -60.0  * PI/180.0;
      ctrl_joint_angle[2] = 20.0 * PI/180.0;
      ctrl_joint_angle[3] = 40.0 * PI/180.0;

      opencr_port.write("joint"            + ',' +
                        ctrl_joint_angle[0] + ',' +
                        ctrl_joint_angle[1] + ',' +
                        ctrl_joint_angle[2] + ',' +
                        ctrl_joint_angle[3] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Gripper_pos(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("grip"  + ',' +
                          "on" + '\n');
      }
      else
      {
        opencr_port.write("grip"  + ',' +
                          "off" + '\n');
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Hand Teaching Controller
*******************************************************************************/
  void Torque_OnOff(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("torque"  + ',' +
                          "off"     + '\n');
      }
      else
      {
        opencr_port.write("torque"  + ',' +
                          "on"      + '\n');
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
  
  public void Motion_Clear(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("get" + ',' +
                        "clear"  + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Make_Joint_Pose(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("get"      + ',' +
                        "pose"     + ',' +
                        motion_num + '\n');

      motion_num++;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Make_Gripper_Pose(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("get"    + ',' +
                          "on"  + '\n');

        gripper.setValue(1.3);
      }
      else
      {
        opencr_port.write("get"     + ',' +
                          "off"  + '\n');

        gripper.setValue(0.0);
      }
      motion_num++;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Motion_Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("hand"     + ',' +
                        "once"  + '\n');
      println("Motion Start!!!");

      motion_num = 0;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Motion_Repeat(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("hand"    + ',' +
                          "repeat"  + '\n');
      }
      else
      {
        opencr_port.write("hand"  + ',' +
                          "stop"  + '\n');;
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("motion"  + ',' +
                        "start"   + '\n');
      println("Motion Start!!!");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Stop(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("motion"  + ',' +
                        "stop"    + '\n');
      println("Motion Stop!!!");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
}
