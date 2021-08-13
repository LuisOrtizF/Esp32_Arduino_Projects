import processing.serial.*;
Serial myPort;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

void setup()
{
  size(600, 500, P3D);
  myPort = new Serial(this, "/dev/ttyUSB0", 9600); // Linux
  textSize(16);                                    // set text size
  textMode(SHAPE);                                 // set text mode to shape
}

void draw()
{
  serialEvent();   // read and parse incoming serial message
  background(255); // set background to white
  lights();

  translate(width/2, height/2); // set position to centre

  pushMatrix();                 // begin object
  
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(-pitch));
  float s2 = sin(radians(-pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);
               
  drawLSM9DS1();

  popMatrix(); // end of object
      
  print(roll);
  print("\t");
  print(pitch);
  print("\t");
  print(yaw);
  println();
}

void serialEvent()
{
  int newLine = 13; // new line character '\n' in ASCII 
  String message;
  
  do 
  {
    message = myPort.readStringUntil(newLine); // read from port until new line
    
    if (message != null) 
    {
      String[] list = split(trim(message), " ");
            
      if (list.length == 3) 
      {
        roll = float(list[0]);  // convert to float
        pitch = float(list[1]); 
        yaw = float(list[2]);
      }
    }
  } while (message != null);
}

void drawLSM9DS1()
{
 
  ///* function contains shape(s) that are rotated with the IMU */
  
  stroke(0);               // set outline colour to black
  fill(0, 75, 150);        // set fill colour to blue
  box(150, 7, 100);        // draw LSM9DS1 board base shape 
  
  // draw X axis
  stroke(0,255,0);
  line(0, 0, 0, 100, 0, 0);
  fill(0,255,0);
  text("X", 110, 0, 0);
  
  // draw Y axis
  stroke(255,0,0);
  line(0, 0, 0, 0, 0, -100);
  fill(255,0,0);
  text("Y", 0,0, -110);
  
  // draw Z axis
  stroke(0,0,255);
  line(0, 0, 0, 0, -100, 0);
  fill(0,0,255);
  text("Z",0,-110,0);
  
  // draw LSM9DS1 CHIP
  translate(0,-7,0);      // set position
  stroke(128);            // set outline colour
  fill(0);                // set fill colour 
  box(30,7,20);           // draw LSM9DS1 CHIP
  
  // draw LSM9DS1 CHIP point
  translate(-10,-3,-6);     // set position
  sphere(2);
  
  // draw 9 contacts on Y- side
  //fill(224, 224, 224);     // silver color
  //noStroke();              // set outline colour
  //translate(-40, -65, 0);
  //for (int i=0; i<9; i++) 
  //{
  //  sphere(4.5);           // draw silver contacts
  //  translate(10, 0, 0);   // set new position
  //}
  
  //// draw 4 contacts on Y+ side
  //fill(224, 224, 224);     // silver color
  //noStroke();              // set outline colour
  //translate(-65, 130, 0);  // set position to contacts
  //for (int i=0; i<4; i++) 
  //{
  //  sphere(4.5);           // draw silver contacts
  //  translate(10, 0, 0);   // set new position
  //}  
}
