/*
Oscilloscope

This example shows how to make oscilloscope using OpenCM9.04 and Processing. 
Processing codes are also available as the below.

1. Pin 0 is conneced to probe(just wire is okay)
2. download this example to OpenCM9.04 and run it.
3. connect OpencM9.04 to PC using USB
4. compile the below Processing sketch and execute it on PC.
5. detected signal can be visually displayed.

*/

void setup(){
  SerialUSB.begin();
  pinMode(0, INPUT_ANALOG);
}

void loop(){
  int val = analogRead(0);
  
  SerialUSB.write( 0xff );//send header packet                                                         
  SerialUSB.write( (val >> 8) & 0xff ); //send high byte                                           
  SerialUSB.write( val & 0xff );  //send low byte
}


//The following code shows the electrical signal visually using Processing
//This sketch program can be run on Processing. 
//https://www.processing.org/

/*
 * Oscilloscope
 * Gives a visual rendering of analog pin 0 in realtime.
 * 
 * This project is part of Accrochages
 * See http://accrochages.drone.ws
 * 
 * (c) 2008 Sofian Audry (info@sofianaudry.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */ 
 
 /*
import processing.serial.*;
 
Serial port;  // Create object from Serial class
int val;      // Data received from the serial port
int[] values;
float zoom;
float center;
float scale = 1;
 
void setup() 
{
  size(1280, 480);
  // Open the port that the board is connected to and use the same speed (9600 bps)
  port = new Serial(this, Serial.list()[0], 9600); // OpenCM9.04 포트를 알고 있을 경우 이렇게도 사용이 가능하다. port = new Serial(this, “COM3”, 9600);
  values = new int[width];
  zoom = 1.0f;
  smooth();
}
 
int getY(int val) {
  return (int)(height/2  -(val-512+center)*scale / 1023.0f * (height - 1));
}
 
int getValue() {
  int value = -1;
  while (port.available() >= 3) {
    if (port.read() == 0xff) {
      value = (port.read() << 8) | (port.read());
    }
  }
  return value;
}
 
void pushValue(int value) {
  for (int i=0; i<width-1; i++)
    values[i] = values[i+1];
  values[width-1] = value;
}
 
void drawLines() {
  stroke(255);
  
  int displayWidth = (int) (width / zoom);
  
  int k = values.length - displayWidth;
  
  int x0 = 0;
  int y0 = getY(values[k]);
  for (int i=1; i<displayWidth; i++) {
    k++;
    int x1 = (int) (i * (width-1) / (displayWidth-1));
    int y1 = getY(values[k]);
    line(x0, y0, x1, y1);
    x0 = x1;
    y0 = y1;
  }
}
 
void drawGrid() {
  stroke(255, 0, 0);
  line(0, height/2, width, height/2);
}
 
void keyReleased() {
  println(key+": "+(int)key);
  switch (key) {
    case '4':
    case '*':
      zoom *= 2.0f;
      println(zoom);
      if ( (int) (width / zoom) <= 1 )
        zoom /= 2.0f;
      break;
    case '6':
    case '/':
      zoom /= 2.0f;
      if (zoom < 1.0f)
        zoom *= 2.0f;
      break;
     case '+':	scale*=2;  break;
     case '-':	scale /= 2;  break;
     
     case '8':	center += 10/scale; break;
     case '2':	center -= 10/scale; break;
     case '5':	center = 0; scale = 1; break;  
  }
}
 
void draw()
{
  background(0);
  drawGrid();
  val = getValue();
  if (val != -1) {
    pushValue(val);
  }
  drawLines();
}
*/