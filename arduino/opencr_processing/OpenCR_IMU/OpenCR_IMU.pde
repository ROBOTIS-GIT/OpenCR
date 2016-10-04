import processing.serial.*;


Serial serial_port = null;        // the serial port


// IMU 
//
float imu_x;
float imu_y;
float imu_z;

String  inString;  

final int VIEW_SIZE_X = 600, VIEW_SIZE_Y = 600;



// Serial
//
Button btn_serial_up;              // move up through the serial port list
Button btn_serial_dn;              // move down through the serial port list
Button btn_serial_connect;         // connect to the selected serial port
Button btn_serial_disconnect;      // disconnect from the serial port
Button btn_serial_list_refresh;    // refresh the serial port list
String serial_list;                // list of serial ports
int serial_list_index = 0;         // currently selected serial port 
int num_serial_ports = 0;          // number of serial ports in the list

int btn_x = 240;
int box_x = 4;
int serial_baud = 115200;// serial port buttons





void setup() 
{
  size(600, 600, P3D);


// create the buttons
  btn_serial_up = new Button("^", btn_x, 10, 40, 20);
  btn_serial_dn = new Button("v", btn_x, 50, 40, 20);
  btn_serial_connect = new Button("Open", btn_x+50, 10, 100, 25);
  btn_serial_list_refresh = new Button("Refresh", btn_x+50, 45, 100, 25);
  
  // get the list of serial ports on the computer
  serial_list = Serial.list()[serial_list_index];
  
  
  // get the number of serial ports in the list
  num_serial_ports = Serial.list().length;
  
  Serial_Init();

/*
  println(Serial.list());                      //포트 리스트 출력
  myPort = new Serial(this, "/dev/tty.usbmodemfa131", 115200);  
  myPort.clear();
  myPort.bufferUntil(10);
*/  
}


void mousePressed() {
  // up button clicked
  if (btn_serial_up.MouseIsOver()) {
    if (serial_list_index > 0) {
      // move one position up in the list of serial ports
      serial_list_index--;
      serial_list = Serial.list()[serial_list_index];
    }
  }
  // down button clicked
  if (btn_serial_dn.MouseIsOver()) {
    if (serial_list_index < (num_serial_ports - 1)) {
      // move one position down in the list of serial ports
      serial_list_index++;
      serial_list = Serial.list()[serial_list_index];
    }
  }
  // Connect button clicked
  if (btn_serial_connect.MouseIsOver()) {
    if (serial_port == null) {
      // connect to the selected serial port
      serial_port = new Serial(this, Serial.list()[serial_list_index], serial_baud);
      btn_serial_connect.label = "Close";
      btn_serial_connect.fill_color = 100;
    }
    else
    {
      if (serial_port != null) {
        // disconnect from the serial port
        serial_port.stop();
        serial_port = null;
        btn_serial_connect.label = "Open";
        btn_serial_connect.fill_color = 218;
      }      
    }
  }
  // Refresh button clicked
  if (btn_serial_list_refresh.MouseIsOver()) {
    // get the serial port list and length of the list
    serial_list = Serial.list()[serial_list_index];
    num_serial_ports = Serial.list().length;
  }
}



void buildBoxShape() 
{
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawCube() {  
  pushMatrix();
    translate(300, 300, 0);
    scale(4,4,4);
    
    rotateY( -imu_z * PI / 180);
    rotateX(  imu_y * PI / 180);
    rotateZ(  imu_x * PI / 180);
    

    buildBoxShape();
    
  popMatrix();
}


// function for drawing a text box with title and contents
void DrawTextBox(String title, String str, int x, int y, int w, int h)
{
  fill(255);
  rect(x, y, w, h);
  fill(0);
  textAlign(LEFT);
  textSize(14);
  text(title, x + 10, y + 10, w - 20, 20);
  textSize(12);  
  text(str, x + 10, y + 40, w - 20, h - 10);
}

// button class used for all buttons
class Button {
  String label;
  float x;    // top left corner x position
  float y;    // top left corner y position
  float w;    // width of button
  float h;    // height of button
  int fill_color;
  
  // constructor
  Button(String labelB, float xpos, float ypos, float widthB, float heightB) {
    label = labelB;
    x = xpos;
    y = ypos;
    w = widthB;
    h = heightB;
    
    fill_color = 218;
  }
  
  // draw the button in the window
  void Draw() {
    fill(fill_color);
    stroke(141);
    rect(x, y, w, h, 10);
    textAlign(CENTER, CENTER);
    fill(0);
    text(label, x + (w / 2), y + (h / 2));
  }
  
  // returns true if the mouse cursor is over the button
  boolean MouseIsOver() {
    if (mouseX > x && mouseX < (x + w) && mouseY > y && mouseY < (y + h)) {
      return true;
    }
    return false;
  }
}


void drawSerialList()
{
  // draw the buttons in the application window
  btn_serial_up.Draw();
  btn_serial_dn.Draw();
  btn_serial_connect.Draw();
  btn_serial_list_refresh.Draw();
  // draw the text box containing the selected serial port
  DrawTextBox("Select Port", serial_list, box_x, 10, 220, 60);
}  



int xPos = 1;         // horizontal position of the graph 
int lastxPos=1;
int lastheight=0;


int draw_box_w = VIEW_SIZE_X-10;
int draw_box_h = 100;
int draw_box_x = 5;
int draw_box_y = VIEW_SIZE_Y - draw_box_h - 10;

int scale_limit_top = 180;
int scale_limit_bottom = -180;

int line_s_x;
int line_s_y;
int line_e_x;
int line_e_y;


int FBufMax = 16;
float FBuf[][];
int FPos[];
int box_center_y = draw_box_y+draw_box_h/2;

color FColor[];


void Serial_Init()
{
  FBuf = new float[FBufMax][draw_box_w];
  FPos = new int[FBufMax];
  FColor = new color[FBufMax];
  
  FColor[0] = #FF0000;
  FColor[1] = #00FF00;
  FColor[2] = #0000FF;
  
  
  
  for( int i=0; i<FBufMax; i++ )
  {
    FPos[i] = 0;
  }
}

void Serial_AddData( int Index, float Data )
{
  if( Index >= FBufMax ) return;
  
  FBuf[Index][ FPos[Index] ] = Data;
  FPos[Index]++;
  FPos[Index] %= draw_box_w;
}

void Serial_Draw()
{
  int i;
  
  fill(0);
  rect( draw_box_x, draw_box_y, draw_box_w, draw_box_h);
  stroke(127,34,255);     //stroke color
  strokeWeight(1);        //stroke wider
  line(line_s_x, line_s_y, line_e_x, line_e_y);       

  stroke(255,0,0);
  strokeWeight(1);  
  line(draw_box_x, box_center_y, draw_box_x+draw_box_w, box_center_y);
  
  for( i=0; i<FBufMax; i++ )
  {
    stroke(FColor[i]);
    Serial_DrawFrame(i);
  } 
}


void Serial_DrawFrame( int Index )
{
  int draw_w;
  int draw_h;
  int draw_x;
  int draw_y;
  
  int y0;
  int y1;
  float Value;
  
  draw_x = draw_box_x;
  draw_y = draw_box_y;
  draw_w = draw_box_w;
  draw_h = draw_box_h;
  
  for(int x=0;x<draw_w-1;x++) 
  {    
    Value = constrain( FBuf[Index][(x+FPos[Index]) % draw_w], scale_limit_bottom, scale_limit_top);
    y0 = (int)map(Value, scale_limit_bottom, scale_limit_top, 0, draw_h);
    
    Value = constrain( FBuf[Index][(x+FPos[Index]+1) % draw_w], scale_limit_bottom, scale_limit_top);
    y1 = (int)map(Value, scale_limit_bottom, scale_limit_top, 0, draw_h);
      // We want y=0 to be at the bottom of the screen,
      // Cartesian-style
    line(draw_x+x, draw_y+draw_h-y0-1,draw_x+x+1, draw_y+draw_h-y1-1);
  } 
}


void draw() {  
  
  background(#000000);
  fill(#ffffff);

  drawSerialList();  
  drawCube();
  Serial_Draw();  
}


//데이터 받아오기
void serialEvent(Serial p) {

  inString = (p.readStringUntil('\n'));
  
  try {
  
    if( inString != null )
    { 
      String[] dataStrings = split(inString, ' ');
      imu_x = float(dataStrings[1]);
      imu_y = float(dataStrings[2]);
      imu_z = float(dataStrings[3]);
      
      Serial_AddData( 0, imu_x );
      Serial_AddData( 1, imu_y );
      Serial_AddData( 2, imu_z );
      
      println( imu_x + " " + imu_y + " " + imu_z );
    }
  } catch (Exception e) {
      println("Caught Exception");
  }
}