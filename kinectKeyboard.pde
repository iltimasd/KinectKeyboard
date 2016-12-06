// - Super Fast Blur v1.1 by Mario Klingemann <http://incubator.quasimondo.com>
// - BlobDetection library

import blobDetection.*;

BlobDetection theBlobDetection;

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

Kinect kinect;
import themidibus.*; //Import the library

MidiBus myBus; // The MidiBus
ArrayList<Integer> bLoc;  //blob Location
// Depth image
int cols=3; //length of grid
int rows=3; //height of grid
int [] squares = new int[cols*rows]; //Track which squares are currently occupied
int [] lastState = new int[cols*rows]; //Track last state to know if we should send a noteOn or Off
int boxsizeW = 640/rows; //sets the size of the box
int boxsizeH = 480/cols; //sets the size of the box

PImage depthImg;

// Which physical height do we care about? Different for each installation
int minDepth =  540;
int maxDepth = 960;
boolean newFrame=false;

void setup()
{
  bLoc = new ArrayList<Integer>();
  for (int i=0; i<squares.length; i++) {
    lastState[i]=0;
  }
  myBus = new MidiBus(this, -1, "loopMIDI Port"); // Create a new MidiBus 
  for (int i=0; i<12*11; i++) {
    myBus.sendNoteOff(1, i, 255); // Send Midi nodeOff for all notes
  }
  kinect = new Kinect(this);
  kinect.initDepth();

  // Blank image
  depthImg = new PImage(kinect.width, kinect.height);
  // Size of applet
  size(640, 480);
  // Capture

  // BlobDetection
 
  theBlobDetection = new BlobDetection(depthImg.width, depthImg.height);
  theBlobDetection.setPosDiscrimination(true);
  theBlobDetection.setThreshold(0.2f); // will detect bright areas whose luminosity > 0.2f;
}


void draw()
{
//initialize array at the start of each loop
  for (int i=0; i<squares.length; i++) {
    squares[i]=0;
  }
//for each blob/location set the corresponding sqaure as occupied
  for (int j = bLoc.size()-1; j >= 0; j--) {
    for (int i=0; i<squares.length; i++) {
      if (bLoc.get(j)==i) {  
        squares[i]=1; //set the array loation as 1(aka true)
        i=squares.length; //Move on the next item in bLoc arrayList
      }
    }
  }
//for each occupied space send a noteOn or Off
  for (int i=0; i<squares.length; i++) {
    if (lastState[i]!=squares[i]) {
      if (squares[i]==1) {
        myBus.sendNoteOn(1, (i+15)*2, 255); // Send a Midi noteOn
//                              ^
//                              +-------------------- Change this expression to change the range of notes in the grid
        lastState[i]=1;
      } else {
        myBus.sendNoteOff(1, (i+15)*2, 255); // Send a Midi nodeOff
        lastState[i]=0;
      }
    }
  }

  purge();
  //compress depth map from given range to a flat black and white image
  int[] rawDepth = kinect.getRawDepth();
  for (int i=0; i < rawDepth.length; i++) {
    if (rawDepth[i] >= minDepth && rawDepth[i] <= maxDepth) {
      depthImg.pixels[i] = color(255);
    } else {
      depthImg.pixels[i] = color(0);
    }
  }

  // Draw the thresholded image
  depthImg.updatePixels();
  image(depthImg, 0, 0);

  fill(0);

  fastblur(depthImg, 2);
  theBlobDetection.computeBlobs(depthImg.pixels);
  drawBlobsAndEdges();

  for (int y = 0; y < rows; y++ ) { //loop for every row of grid
    for (int x = 0; x < cols; x++) { //loop for every col of grid
      int x2 = x * boxsizeW; //intializing the x coordinate of each box
      int y2 = y * boxsizeH; //intializing the y coordinate of each box




      if (squares[((y*cols)+x)]==1) { //If mouse is on specified square (TRUE or 1)
        fill(255, 0, 0, 125); //fill with red
      } else if (squares[(y*cols+x)]==0) { //fill with white
        fill(0, 0);
      }
      rect(x2, y2, boxsizeW, boxsizeH); //drawing the red box AFTER THE FILL. IMPORTANT
    }
  }
}

void keyPressed() {
  for (int i=0; i<12*11; i++) {
    myBus.sendNoteOff(1, i, 255); // Send Midi nodeOff for all notes
  }
  exit(); //byeeeeee
}

//delete the arraylist. We do this so that the arraylist can be made to the same size as the number of blobs.
//perhaps no elegant, but I have yet to find a stable way to accurately track the entering and leaving of blobs.
//so nearly all of this code computes everything fresh every loop.
void purge() {
  for (int i = bLoc.size() - 1; i >= 0; i--) {
    bLoc.remove(i);
  }
}

//do the blob thing
void drawBlobsAndEdges()
{
  noFill();
  Blob b;
  for (int n=0; n<theBlobDetection.getBlobNb(); n++)
  {
    b=theBlobDetection.getBlob(n);
    if (b!=null)
    {
      for (int y = 0; y < rows; y++ ) { //loop for every row of grid
        for (int x = 0; x < cols; x++) { //loop for every col of grid
          int x2 = x * boxsizeW; //intializing the x coordinate of each box
          int y2 = y * boxsizeH; //intializing the y coordinate of each box

          //if the center of the bounding box of the blob is in a square,
          if (b.xMin*width+(b.w*width/2)>x2 &&b.xMin*width+(b.w*width/2)<x2+boxsizeW && b.yMin*height+(b.h*height)/2>y2 && b.yMin*height+(b.h*height)/2<y2+boxsizeH) {
            bLoc.add(0, y*cols+x); // add(*add because itll be an empty arraylist at the start of this function every loop) it the bLoc arraylist
          }
        }
      }
      strokeWeight(1);
      stroke(255, 0, 255);
      ellipse(
        b.xMin*width+(b.w*width/2), b.yMin*height+(b.h*height)/2, 3, 3); //draw the center
    }
  }
 // println(theBlobDetection.getBlobNb());  //making sure this and the next are the same
  //println(bLoc.size());
}

//this was SUPPOSED to reject small blobs, havent gotten it to work
boolean newBlobDetectedEvent (Blob b) { 
  int w = (int)(b.w*width );
  int h = (int)(b.h*width );
  println("hi");
  if (w >= 20 || h >= 20) {
    return true;
  }
  return false;
}

// ==================================================
// Super Fast Blur v1.1
// by Mario Klingemann 
// <http://incubator.quasimondo.com>
// ==================================================
void fastblur(PImage img, int radius)
{
  if (radius<1) {
    return;
  }
  int w=img.width;
  int h=img.height;
  int wm=w-1;
  int hm=h-1;
  int wh=w*h;
  int div=radius+radius+1;
  int r[]=new int[wh];
  int g[]=new int[wh];
  int b[]=new int[wh];
  int rsum, gsum, bsum, x, y, i, p, p1, p2, yp, yi, yw;
  int vmin[] = new int[max(w, h)];
  int vmax[] = new int[max(w, h)];
  int[] pix=img.pixels;
  int dv[]=new int[256*div];
  for (i=0; i<256*div; i++) {
    dv[i]=(i/div);
  }

  yw=yi=0;

  for (y=0; y<h; y++) {
    rsum=gsum=bsum=0;
    for (i=-radius; i<=radius; i++) {
      p=pix[yi+min(wm, max(i, 0))];
      rsum+=(p & 0xff0000)>>16;
      gsum+=(p & 0x00ff00)>>8;
      bsum+= p & 0x0000ff;
    }
    for (x=0; x<w; x++) {

      r[yi]=dv[rsum];
      g[yi]=dv[gsum];
      b[yi]=dv[bsum];

      if (y==0) {
        vmin[x]=min(x+radius+1, wm);
        vmax[x]=max(x-radius, 0);
      }
      p1=pix[yw+vmin[x]];
      p2=pix[yw+vmax[x]];

      rsum+=((p1 & 0xff0000)-(p2 & 0xff0000))>>16;
      gsum+=((p1 & 0x00ff00)-(p2 & 0x00ff00))>>8;
      bsum+= (p1 & 0x0000ff)-(p2 & 0x0000ff);
      yi++;
    }
    yw+=w;
  }

  for (x=0; x<w; x++) {
    rsum=gsum=bsum=0;
    yp=-radius*w;
    for (i=-radius; i<=radius; i++) {
      yi=max(0, yp)+x;
      rsum+=r[yi];
      gsum+=g[yi];
      bsum+=b[yi];
      yp+=w;
    }
    yi=x;
    for (y=0; y<h; y++) {
      pix[yi]=0xff000000 | (dv[rsum]<<16) | (dv[gsum]<<8) | dv[bsum];
      if (x==0) {
        vmin[y]=min(y+radius+1, hm)*w;
        vmax[y]=max(y-radius, 0)*w;
      }
      p1=x+vmin[y];
      p2=x+vmax[y];

      rsum+=r[p1]-r[p2];
      gsum+=g[p1]-g[p2];
      bsum+=b[p1]-b[p2];

      yi+=w;
    }
  }
}
