//------------------------------------------------------------------------------
// COORDINATE SYSTEM:
//------------------------------------------------------------------------------
// (0,0) is center of drawing surface.  -y is up.  -x is left
// All lengths are in cm unless otherwise stated.



//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
// Adafruit motor driver library
#include <AFMotor.h>
// Default servo library
#include <Servo.h> 



//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
// NEMA17 are 200 steps (1.8 degrees) per turn.  If a spool is 0.8 diameter
// then it is 2.5132741228718345 circumference, and
// 2.5132741228718345 / 200 = 0.0125663706 thread moved each step.
// adafruit can handle ~1500RPM.  The Mega's inner loop for calculating
// when to move motors can't keep up at that speed and movement gets stupid.
// A faster microprocessor could take full advantage of the machine.
#define STEPS_PER_TURN  (200.0)
#define SPOOL_DIAMETER  (0.9)
#define RPM             (250.0)
// how fast can the plotter accelerate in a straight line?  cm/s/s.
#define ACCELERATION    (3.0)


// Dan's config
#define X_SEPARATION    (28.2)  // Distance between stepper shaft centers.
// Distance from center to bottom of drawing area.
#define LIMYMAX         (20.0)
// Distance from center to top of drawing area.  That's the line between the 
// two stepper motors.  The plotter cannot physically reach this line - it
// would require infinite tensile strength.
#define LIMYMIN         (-21.5)

// which motor is on which pin?
#define M1_PIN          (2)
#define M2_PIN          (1)

// which way are the spools wound, relative to motor movement?
#define REEL_IN         FORWARD
#define REEL_OUT        BACKWARD

// adjust the scaling to improve the accuracy of the output
#define SCALE_X         (6.0/6.0)
#define SCALE_Y         (6.0/6.0)

/*
// Paul's config
#define X_SEPARATION    (60.0)  // Distance between stepper centers - pen holder width.
#define LIMYMAX         (15.0)
#define LIMYMIN         (-26.0)
#define M1_PIN          (2)
#define M2_PIN          (1)
#define REEL_IN         BACKWARD
#define REEL_OUT        FORWARD
#define SCALE_X         (6.0/5.5)
#define SCALE_Y         (6.0/8.0)
*/

// define only one of these options
#define STEP_MODE       SINGLE
//#define STEP_MODE       DOUBLE
//#define STEP_MODE       INTERLEAVE
//#define STEP_MODE       MICROSTEP


// servo angles for pen control
#define PEN_UP_ANGLE    (90)
#define PEN_DOWN_ANGLE  (10)  // Some steppers don't like 0 degrees


// Uncomment this line if you have a smaller chip like atmel128.
//#define SMALL_FOOTPRINT (1)
// Comment out this line to silence most serial output.
// Cannot be on if you want a small footprint.
//#define VERBOSE         (1)


// ************************************************
// *** Do not change constants below this line. ***
// ************************************************

#define SPOOL_CIRC      (SPOOL_DIAMETER*PI)  // circumference
#define TPS             (SPOOL_CIRC/STEPS_PER_TURN)  // thread per step
#define MAXVELOCITY     (RPM*SPOOL_CIRC/60.0)  // cm/s

// limits plotter can move.
#define LIMXMAX         ( X_SEPARATION*0.5)
#define LIMXMIN         (-X_SEPARATION*0.5)

// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)

// for pen actions
#define PEN_UP          (1)
#define PEN_DOWN        (0)

// Serial communication bitrate
#define BAUD            (57600)
// Maximum length of serial input message.
#define MAX_BUF         (64)



//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------
// Initialize Adafruit stepper controller
AF_Stepper m1((int)STEPS_PER_TURN, M2_PIN);  // to motor port #1 (M1 and M2)
AF_Stepper m2((int)STEPS_PER_TURN, M1_PIN);  // to motor port #2 (M3 and M4)

Servo s1;

// plotter position.
static float posx;
static float posy;

// acceleration.
static float accel=ACCELERATION;
static float maxvel=MAXVELOCITY;

// pen state
static int ps;

// time values
static float t;   // since board power on
static float dt;  // since last tick

// Diameter of line made by plotter
static float tool_diameter=0.05;  

// Serial comm reception
char buffer[MAX_BUF];  // Serial buffer
int sofar;             // Serial buffer progress



//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void showLimits() {
  Serial.print("(");
  Serial.print(LIMXMIN);
  Serial.print(",");
  Serial.print(LIMYMIN);
  Serial.print(") - {");
  Serial.print(LIMXMAX);
  Serial.print(",");
  Serial.print(LIMYMAX);
  Serial.println(")");
}


//------------------------------------------------------------------------------
void error(int r) {
  if(r!=0) {
    Serial.print("Error: code ");
    Serial.println(r);
  }
}


//------------------------------------------------------------------------------
// increment internal clock
void tick() {
  float nt=((float)millis())*0.001;
  dt = nt-t;  // time since last tick, in seconds
  t=nt;
}

//------------------------------------------------------------------------------
// Inverse Kinematics - turns XY coordinates into lengths L1,L2
void IK(float x,float y,float &l1, float &l2) {
  // find length to M1
  float dy = y - LIMYMIN;
  float dx = x - LIMXMIN;
  l1 = sqrt(dx*dx+dy*dy);
  // find length to M2
  dx = x - LIMXMAX;
  l2 = sqrt(dx*dx+dy*dy);
}


//------------------------------------------------------------------------------
// Forward Kinematics - turns L1,L2 lengths into XY coordinates
// use law of cosines,
// theta = acos((a*a+b*b-c*c)/(2*a*b));
// to find angle between M1M2 and M1P
// where P is the plotter position
void FK(float l1, float l2,float &x,float &y) {
  float a=l1;
  float b=X_SEPARATION;
  float c=l2;
  
  // slow, uses trig
  float theta = acos((a*a+b*b-c*c)/(2.0*a*b));
  x = cos(theta)*l1 + LIMXMIN;
  y = sin(theta)*l1 + LIMYMIN;
}


//------------------------------------------------------------------------------
float interpolate(float start,float diff,float tsum,float time) {
  return start + diff * (tsum/time);
}


//------------------------------------------------------------------------------
void distance_to_time(float len,float &t1,float &t2,float &t3) {
  float tmaxv = maxvel / accel;
  float d1=0.5 * accel * tmaxv * tmaxv;
  float a = d1 * 2;
  
  if(len>a) {
    t1=tmaxv;
    t3 = tmaxv*2 + (len-a) / maxvel;
    t2 = t3-tmaxv;
  } else {
    // len = 0.5 * a * t * t
    // so t = sqrt( len*2.0 / a );
    // but we know we're accelerating to len/2,
    // and then decelerating to len so
    t3 = 2.0 * sqrt( len / accel );
    t1=t2=t3*0.5;
  }
}


//------------------------------------------------------------------------------
float interpolate2(float p0,float p3,float t,float t1,float t2,float t3,float len) {
  float d = p3-p0;
  if(d==0) return p0;

  if(t<0) return p0;
    
  d/=len;
   
  if(t<t1) {
    float d1=0.5 * accel * t * t;
    return p0 + (d1)*d;
  }
  
  if(t<t2) {
    float d1=0.5 * accel * t1 * t1;
    float d2=maxvel * (t-t1);
    return p0 + (d1+d2)*d;
  }
  
  if(t<t3) {
    float t4 = abs(t-t3);
    float d3=0.5 * accel * t4 * t4;
    return p3 - (d3)*d;
  }

  //else if(t>t3)
  return p3;
}


//------------------------------------------------------------------------------
// Turns the motors so that the real robot strings match our simulation.
// len1 & len2 are the current string lengths.
// nlen1 & nlen2 are the new string lengths.
void adjustStringLengths(float &len1,float &len2,float nlen1,float nlen2) {
  do {
    // is the change in length > one step?
    if(nlen1<=len1-TPS) {  // it is shorter.
      m1.step(1,REEL_IN,STEP_MODE);
      len1-=TPS;
      continue;
    } else if(nlen1>=len1+TPS) {  // it is longer.
      m1.step(1,REEL_OUT,STEP_MODE);
      len1+=TPS;
      continue;
    }
    // is the change in length > one step?
    if(nlen2<=len2-TPS) {  // it is shorter.
      m2.step(1,REEL_IN,STEP_MODE);
      len2-=TPS;
      continue;
    } else if(nlen2>=len2+TPS) {  // it is longer.
      m2.step(1,REEL_OUT,STEP_MODE);
      len2+=TPS;
      continue;
    }
    // did the move somehow step more than once?
    // this is the only way to be sure.
  } while(0);
}


//------------------------------------------------------------------------------
// This method assumes the limits have already been checked.
void line(float x,float y) {
  float dx = x - posx;  // delta
  float dy = y - posy;
  float len = sqrt(dx*dx+dy*dy);
  float t1;
  float t2;
  float time;
  float tsum = 0;
  float len1,nlen1,nx;
  float len2,nlen2,ny;
  char did_step;

  // get travel time
  //time = len / maxvel;
  distance_to_time(len,t1,t2,time);
   
  IK(posx,posy,len1,len2);
  
#ifdef VERBOSE
  Serial.print("x=");       Serial.println(x);
  Serial.print("y=");       Serial.println(y);
  Serial.print("posx=");    Serial.println(posx);
  Serial.print("posy=");    Serial.println(posy);
  Serial.print("len1=");    Serial.println(len1);
  Serial.print("len2=");    Serial.println(len2);
  Serial.print("dx=");      Serial.println(dx);
  Serial.print("dy=");      Serial.println(dy);
  Serial.print("len=");     Serial.println(len);
  Serial.print("maxvel=");  Serial.println(maxvel);
  Serial.print("time=");    Serial.println(time);
#endif
  
  tick();
  float tstart=t;
  
  do {
    tick();
    tsum = t-tstart;

    // find where the plotter will be at tsum seconds
    nx = interpolate2(posx,x,tsum,t1,t2,time,len);
    ny = interpolate2(posy,y,tsum,t1,t2,time,len);
    //nx=interpolate(posx,dx,tsum,time);
    //ny=interpolate(posy,dy,tsum,time);
       
    // get the new string lengths
    IK(nx,ny,nlen1,nlen2);

    adjustStringLengths(len1,len2,nlen1,nlen2);

#ifdef VERBOSE
    Serial.print(tsum);    Serial.print('\t');
    Serial.print(nx);      Serial.print(',');
    Serial.print(ny);      Serial.print('\t');
    Serial.print(len1);    Serial.print(',');
    Serial.print(len2);    Serial.print('\n');
#endif

  } while(tsum<time);

  posx=x;
  posy=y;

#ifdef VERBOSE  
  Serial.println("Done.");
#endif
}


//------------------------------------------------------------------------------
// checks against the robot limits before attempting to move
int lineSafe(float x,float y) {
  if(outsideLimits(x,y)) return 1;
  
  line(x,y);
  return 0;
}


//------------------------------------------------------------------------------
// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = posx - cx;
  float dy = posy - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;

  float temp_maxvel=maxvel;
  maxvel/=PI;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  theta=abs(theta);
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = theta * radius;

  // get travel time
  //float time = len / maxvel;
  float t1;
  float t2;
  float time;
  distance_to_time(len,t1,t2,time);
  
  Serial.print("a1=");      Serial.println(angle1);
  Serial.print("a2=");      Serial.println(angle2);
  Serial.print("theta=");   Serial.println(theta*180.0/PI);
#ifdef VERBOSE
  Serial.print("x=");       Serial.println(x);
  Serial.print("y=");       Serial.println(y);
  Serial.print("posx=");    Serial.println(posx);
  Serial.print("posy=");    Serial.println(posy);
  Serial.print("radius=");  Serial.println(radius);
  Serial.print("len=");     Serial.println(len);
  Serial.print("time=");    Serial.println(time);
#endif

  float len1,nlen1,nx;
  float len2,nlen2,ny;
  char did_step;
  float tsum;

  IK(posx,posy,len1,len2);
  tick();
  float tstart=t;
 
  do {
    tick();
    tsum=t-tstart;

    // find where the plotter will be at tsum seconds
    float angle3 = interpolate2(angle1,angle2,tsum,t1,t2,time,len);
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;

    // get the new string lengths
    IK(nx,ny,nlen1,nlen2);

    adjustStringLengths(len1,len2,nlen1,nlen2);
   
#ifdef VERBOSE
    Serial.print(tsum);    Serial.print('\t');
    Serial.print(nx);      Serial.print(',');
    Serial.print(ny);      Serial.print('\t');
    Serial.print(len1);    Serial.print(',');
    Serial.print(len2);    Serial.print('\n');
#endif

  } while(tsum < time);
  
  posx=x;
  posy=y;
  maxvel=temp_maxvel;
  
#ifdef VERBOSE  
  Serial.println("Done.");
#endif
}


//------------------------------------------------------------------------------
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


//------------------------------------------------------------------------------
// is the point (dx,dy) in the arc segment subtended by angle1,angle2?
// return 0 if it is not.
int pointInArc(float dx,float dy,float angle1,float angle2,float dir) {
  float angle3=atan3(-dy,dx);  // atan2 expects +y to be up, so flip the sign
  
#ifdef VERBOSE 
  Serial.print("C:");  Serial.print(dx);
  Serial.print(",");   Serial.print(dy);
  Serial.print("=");   Serial.println(angle3*180.0/PI);
#endif

  if(dir==ARC_CW) {
    if(angle1<angle2) angle1+=PI*2;
#ifdef VERBOSE 
  Serial.print("CW");
  Serial.print(angle1*180.0/PI);  Serial.print(" < ");
  Serial.print(angle3*180.0/PI);  Serial.print(" < ");
  Serial.print(angle2*180.0/PI);  Serial.println("?");
#endif
    if(angle2<=angle3 && angle3<=angle1) return 1;
  } else {
    if(angle2<angle1) angle2+=PI*2;
#ifdef VERBOSE 
  Serial.print("CCW");
  Serial.print(angle2*180.0/PI);  Serial.print(" > ");
  Serial.print(angle3*180.0/PI);  Serial.print(" > ");
  Serial.print(angle1*180.0/PI);  Serial.println("?");
#endif
    if(angle1<=angle3 && angle3<=angle2) return 2;
  }

  return 0;
}


//------------------------------------------------------------------------------
// ...checks start & end radius match
// ...checks against the envelope limits
int canArc(float cx,float cy,float x,float y,float dir) {
#ifdef VERBOSE  
  showLimits();
  Serial.print("end=");  Serial.print(x);
  Serial.print(",");     Serial.println(y);
#endif
  if(outsideLimits(x,y)) return 1;

  float a=x-cx;
  float b=y-cy;
  float c=posx-cx;
  float d=posy-cy;
  float r1=sqrt(a*a+b*b);
  float r2=sqrt(c*c+d*d);
  
  if( abs(r1-r2) > 0.001 ) {
    Serial.print("r1=");  Serial.println(r1);
    Serial.print("r2=");  Serial.println(r2);

    return 2;  // radii don't match
  }
  
  float angle1=atan3(d,c);
  float angle2=atan3(b,a);

#ifdef VERBOSE
  Serial.print("A:");  Serial.print(c);
  Serial.print(",");   Serial.print(d);
  Serial.print("=");   Serial.println(angle1*180.0/PI);
  Serial.print("B:");  Serial.print(a);
  Serial.print(",");   Serial.print(b);
  Serial.print("=");   Serial.println(angle2*180.0/PI);
  Serial.print("r=");  Serial.println(r1);
#endif

  if(cx+r1>LIMXMAX) {
    // find the two points of intersection, see if they are inside the arc
    float dx=LIMXMAX-cx;
    float dy=sqrt(r1*r1-dx*dx);
    if(pointInArc(dx, dy,angle1,angle2,dir)) return 3;
    if(pointInArc(dx,-dy,angle1,angle2,dir)) return 4;
  }
  if(cx-r1<LIMXMIN) {
    // find the two points of intersection, see if they are inside the arc
    float dx=LIMXMIN-cx;
    float dy=sqrt(r1*r1-dx*dx);
    if(pointInArc(dx, dy,angle1,angle2,dir)) return 5;
    if(pointInArc(dx,-dy,angle1,angle2,dir)) return 6;
  }
  if(cy+r1>LIMYMAX) {
    // find the two points of intersection, see if they are inside the arc
    float dy=LIMYMAX-cy;
    float dx=sqrt(r1*r1-dy*dy);
    if(pointInArc( dx,dy,angle1,angle2,dir)) return 7;
    if(pointInArc(-dx,dy,angle1,angle2,dir)) return 8;
  }
  if(cy-r1<LIMYMIN) {
    // find the two points of intersection, see if they are inside the arc
    float dy=LIMYMIN-cy;
    float dx=sqrt(r1*r1-dy*dy);
    if(pointInArc( dx,dy,angle1,angle2,dir)) return 9;
    if(pointInArc(-dx,dy,angle1,angle2,dir)) return 10;
  }

  return 0;
}


//------------------------------------------------------------------------------
// before attempting to move...
// ...checks start & end radius match
// ...checks against the envelope limits
int arcSafe(float cx,float cy,float x,float y,float dir) {
  int r=canArc(cx,cy,x,y,dir);
  if(r==0) {
    arc(cx,cy,x,y,dir);
  }
  return r;
}


//------------------------------------------------------------------------------
#ifndef SMALL_FOOTPRINT
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void testArcs() {
  int r;
  float x,y;
  
  Serial.println(atan3( 1, 1)*180.0/PI);
  Serial.println(atan3( 1,-1)*180.0/PI);
  Serial.println(atan3(-1,-1)*180.0/PI);
  Serial.println(atan3(-1, 1)*180.0/PI);
  
  x=LIMXMAX*0.75;
  y=LIMYMIN*0.50;

  Serial.println("arcs inside limits, center inside limits (should not fail)");
  teleport(x,0);
  error(canArc(0,0,-x,0,ARC_CCW));
  error(canArc(0,0, x,0,ARC_CCW));
  error(canArc(0,0,-x,0,ARC_CW));
  error(canArc(0,0, x,0,ARC_CW));

  Serial.println("arcs outside limits, center inside limits (should fail)");
  x=x*5;
  teleport(x,0);
  error(canArc(0,0,-x,0,ARC_CCW));
  error(canArc(0,0, x,0,ARC_CCW));
  error(canArc(0,0,-x,0,ARC_CW));
  error(canArc(0,0, x,0,ARC_CW));

  x=LIMXMAX*0.75;
  y=LIMYMIN*0.50;
  Serial.println("arcs outside limits, arc center outside limits (should fail)");
  teleport(x,y);
  error(canArc(x,0,-x,y,ARC_CW));

  // LIMXMAX boundary test
  x=LIMXMAX*0.75;
  y=LIMYMIN*0.50;
  Serial.println("CCW through LIMXMAX (should fail)");      teleport(x, y);  error(canArc(x,0, x,-y, ARC_CCW));
  Serial.println("CW avoids LIMXMAX (should not fail)");    teleport(x, y);  error(canArc(x,0, x,-y, ARC_CW));
  Serial.println("CW through LIMXMAX (should fail)");       teleport(x,-y);  error(canArc(x,0, x, y, ARC_CW));
  Serial.println("CCW avoids LIMXMAX (should not fail)");   teleport(x,-y);  error(canArc(x,0, x, y, ARC_CCW));
  // LIMXMIN boundary test
  x=LIMXMIN*0.75;
  y=LIMYMIN*0.50;
  Serial.println("CW through LIMXMIN (should fail)");       teleport(x, y);  error(canArc(x,0, x,-y, ARC_CW));
  Serial.println("CCW avoids LIMXMIN (should not fail)");   teleport(x, y);  error(canArc(x,0, x,-y, ARC_CCW));
  Serial.println("CCW through LIMXMIN (should fail)");      teleport(x,-y);  error(canArc(x,0, x, y, ARC_CCW));
  Serial.println("CW avoids LIMXMIN (should not fail)");    teleport(x,-y);  error(canArc(x,0, x, y, ARC_CW));
  // LIMYMAX boundary test
  x=LIMXMAX*0.50;
  y=LIMYMAX*0.75;
  Serial.println("CW through LIMYMAX (should fail)");       teleport( x,y);  error(canArc(0,y,-x, y, ARC_CW));
  Serial.println("CCW avoids LIMYMAX (should not fail)");   teleport( x,y);  error(canArc(0,y,-x, y, ARC_CCW));
  Serial.println("CCW through LIMYMAX (should fail)");      teleport(-x,y);  error(canArc(0,y, x, y, ARC_CCW));
  Serial.println("CW avoids LIMYMAX (should not fail)");    teleport(-x,y);  error(canArc(0,y, x, y, ARC_CW));
  // LIMYMIN boundary test
  x=LIMXMAX*0.50;
  y=LIMYMIN*0.75;
  Serial.println("CCW through LIMYMIN (should fail)");      teleport( x,y);  error(canArc(0,y,-x, y, ARC_CCW));
  Serial.println("CW avoids LIMYMIN (should not fail)");    teleport( x,y);  error(canArc(0,y,-x, y, ARC_CW));
  Serial.println("CW through LIMYMIN (should fail)");       teleport(-x,y);  error(canArc(0,y, x, y, ARC_CW));
  Serial.println("CCW avoids LIMYMIN (should not fail)");   teleport(-x,y);  error(canArc(0,y, x, y, ARC_CCW));
}


//------------------------------------------------------------------------------
// Show off line and arc movement.  This is the test pattern.
void demo() {
  // square
  Serial.println("> L 0,-2");              line( 0,-2);
  Serial.println("> L-2,-2");              line(-2,-2);
  Serial.println("> L-2, 2");              line(-2, 2);
  Serial.println("> L 2, 2");              line( 2, 2);
  Serial.println("> L 2,-2");              line( 2,-2);
  Serial.println("> L 0,-2");              line( 0,-2);
  // arc
  Serial.println("> A 0,-4,0,-6,ARC_CW");  arc(0,-4,0,-6,ARC_CW);
  Serial.println("> A 0,-4,0,-2,ARC_CCW"); arc(0,-4,0,-2,ARC_CCW);
  // square
  Serial.println("> L 0,-4");              line( 0,-4);
  Serial.println("> L 4,-4");              line( 4,-4);
  Serial.println("> L 4, 4");              line( 4, 4);
  Serial.println("> L-4, 4");              line(-4, 4);
  Serial.println("> L-4,-4");              line(-4,-4);
  Serial.println("> L 0,-4");              line( 0,-4);
  // square
  Serial.println("> L 0,-6");              line( 0,-6);
  Serial.println("> L-6,-6");              line(-6,-6);
  Serial.println("> L-6, 6");              line(-6, 6);
  Serial.println("> L 6, 6");              line( 6, 6);
  Serial.println("> L 6,-6");              line( 6,-6);
  Serial.println("> L 0,-6");              line( 0,-6);
  // large circle
  Serial.println("> A 0,0,0, 6,ARC_CW");   arc(0,0, 0, 6,ARC_CW);
  Serial.println("> A 0,0,0,-6,ARC_CW");   arc(0,0, 0,-6,ARC_CW);

  // triangle
  Serial.println("> L  5.196, 3");         line( 5.196, 3);
  Serial.println("> L -5.196, 3");         line(-5.196, 3);
  Serial.println("> L 0,-6");              line( 0,-6);

  // halftones
  Serial.println("> L -6,-6");             line(-6,-6);
  Serial.println("> L -6, 8");             line(-6, 8);

  int i;
  for(i=0;i<12;++i) {
    Serial.print("> H 1, ");
    Serial.println((float)i/11.0);
    halftone(1,(float)i/11.0);
  }

  // return to origin
  //Serial.println("> L 6, 6");              line( 6, 6);
  Serial.println("> CENTER");              line( 0, 0);
}


//------------------------------------------------------------------------------
void testKinematics(float x,float y) {
  Serial.println("-- TEST KINEMATICS --");
  teleport(x,y);
  float a,b,xx,yy;
  IK(posx,posy,a,b);
  FK(a,b,xx,yy);
  Serial.print("x=");  Serial.print(x);
  Serial.print("\ty=");  Serial.print(y);
  Serial.print("\ta=");  Serial.print(a);
  Serial.print("\tb=");  Serial.print(b);
  Serial.print("\txx=");  Serial.print(xx);
  Serial.print("\tyy=");  Serial.println(yy);
}



//------------------------------------------------------------------------------
void testInterpolate2() {
  Serial.println("-- TEST INTERPOLATE2 --");
  float start=0;
  float end=5;
  float t1,t2,t3;
  
  distance_to_time(end-start,t1,t2,t3);
  for(float t=0;t<t3;t+=0.1) {
    float v=interpolate2(start,end,t,t1,t2,t3,end-start);
    Serial.println(v);
  }
}


//------------------------------------------------------------------------------
void testAcceleration() {
  Serial.println("-- TEST ACCELERATION --");
  Serial.print("maxvel=");  Serial.println(maxvel);

  float i;
  float a=10;
  float b=0;
  float c;
  for(i=3;i<maxvel;i+=0.5) {
    delay(2000);
    accel=i;
    Serial.println(accel);
    line(a,0);
    c=b;
    b=a;
    a=c;
  }
}


//------------------------------------------------------------------------------
void testMaxVel() {
  Serial.println("-- TEST MAX VELOCITY --");
  Serial.print("ACCEL=");  Serial.println(ACCELERATION);
  Serial.print("MAXVELOCITY=");  Serial.println(MAXVELOCITY);

  float i;
  float a=10;
  float b=-10;
  float c;

  line(b,0);

  for(i=5;i<MAXVELOCITY;i+=1) {
    delay(2000);
    maxvel=i;
    Serial.println(maxvel);
    line(a,0);
    c=b;
    b=a;
    a=c;
  }
  line(0,0);
}


//------------------------------------------------------------------------------
#endif  // SMALL_FOOTPRINT
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
void teleport(float x,float y) {
  posx=x;
  posy=y;
}


//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// checks against the robot limits before attempting to move
int teleportSafe(float x,float y) {
  if(outsideLimits(x,y)) return 1;
  teleport(x,y);
  return 0;
}


//------------------------------------------------------------------------------
// halftone generator
// This method assumes the limits have already been checked.
// This method assumes posx, posy is in the middle left of the halftone area
// size - width and height of area to fill.
// fill - [0...1], 0 being least fill and 1 being most fill
//
// plotter will travel left to right filling area from (x1,y1-size/2) to
// (x1+size,y1+size/2) with zigzags.
void halftone(float size,float fill) {
  float ymin=posy-(size*0.5);
  float ymax=posy+(size*0.5);
  
  float max_lines = size / tool_diameter;
  int infill = floor( max_lines * fill );

  Serial.print("size=");        Serial.println(size);
  Serial.print("fill=");        Serial.println(fill);
  Serial.print("max_lines=");   Serial.println(max_lines);
  Serial.print("infill=");      Serial.println(infill);

  // Save starting location because line() changes posx,posy!
  float ox=posx;
  float oy=posy;
  
  if(infill>1) {
    float step = size / (float)(infill);
    Serial.print("step=");          Serial.println(step);
    
    float x2=ox;
    float y2=oy;
    
    for( int i=0; i<infill; ++i ) {
      x2 += step;
      y2 = (i%2)? ymin : ymax;  // the zig-zag effect
      line(x2,y2);  
    }
  }

  // return to the middle of the other side of the halftone square
  line(ox+size,oy);
}


//------------------------------------------------------------------------------
void help() {
  Serial.println("== DRAWBOT - 2012 Feb 15 - dan@marginallyclever.com ==");
  Serial.println("All commands end with a semi-colon.");
  Serial.println("help; - display this message");
  Serial.println("where; - display current virtual coordinates");
  Serial.println("limits; - display maximum distance plotter can move");
#ifndef SMALL_FOOTPRINT
  Serial.println("demo; - run a hardcoded script");
#endif  // SMALL_FOOTPRINT
  Serial.print("teleport; - move the virtual plotter,");
  Serial.println(" don't move the physical plotter.");
  Serial.println("pen (0|1); - pen down|up");
  Serial.println("line [x(float)] [y(float)]; - draw line to x,y");
  Serial.print("arc a(float) b(float) [x(float)] [y(float)] [d(-1|1)];");
  Serial.println(" - draw an arc around a,b to x,y. d is counter|clockwise");
  Serial.print("> ");
}


//------------------------------------------------------------------------------
void where() {
  Serial.print("(");
  Serial.print(posx);
  Serial.print(",");
  Serial.print(posy);
  Serial.print(") ");
  Serial.println(ps==PEN_UP_ANGLE?"UP":"DOWN");
}


//------------------------------------------------------------------------------
void processCommand() {
  if(!strncmp(buffer,"help",4)) {
    help();
  } else if(!strncmp(buffer,"where",5)) {
    where();
  } else if(!strncmp(buffer,"limits",6)) {
    showLimits();
#ifndef SMALL_FOOTPRINT
  } else if(!strncmp(buffer,"demo",4)) {
    demo();
#endif
  } else if(!strncmp(buffer,"pen",3)) {
    int nps=ps;
    
    char *ptr=strchr(buffer,' ')+1;
    if(ptr<buffer+sofar) {
      nps=atoi(ptr);
    }
 
    pen(nps);
  } else if(!strncmp(buffer,"teleport",8)) {
    float xx=posx;
    float yy=posy;
  
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'x': case 'X': xx=atof(ptr+1);  break;
      case 'y': case 'Y': yy=atof(ptr+1);  break;
      default: ptr=0; break;
      }
    }
 
    teleportSafe(xx,yy);
  } else if(!strncmp(buffer,"line",4)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx=posx;
    float yy=posy;
  
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'x': case 'X': xx=atof(ptr+1);  break;
      case 'y': case 'Y': yy=atof(ptr+1);  break;
      default: ptr=0; break;
      }
    }
 
    error(lineSafe(xx,yy));
  } else if(!strncmp(buffer,"arc",3)) {
    // several optional float parameters.
    // then calls a method to do something with those parameters.
    float xx=posx;
    float yy=posy;
    float aa=0;
    float bb=0;
    char found_a=0;
    char found_b=0;
    float dd=1;
  
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'a': case 'A': aa=atof(ptr+1);  found_a=1; break;
      case 'b': case 'B': bb=atof(ptr+1);  found_b=1; break;
      case 'x': case 'X': xx=atof(ptr+1);  break;
      case 'y': case 'Y': yy=atof(ptr+1);  break;
      case 'd': case 'D': dd=atof(ptr+1);  break;
      default: ptr=0; break;
      }
    }

    if(found_a==1 && found_b==1) {
      error(arcSafe(aa,bb,xx,yy,dd));
    } else {
      Serial.println("Error: Center (a,b) must be specified.");
    }
  }
}


//------------------------------------------------------------------------------
void setup() {
  // start communications
  Serial.begin(BAUD);
  Serial.println("== HELLO WORLD ==");
  sofar=0;

  // set the stepper speed
  m1.setSpeed(RPM);
  m2.setSpeed(RPM);
  // servo should be on SER1, pin 10.
  s1.attach(10);
  pen(PEN_UP);

#ifndef SMALL_FOOTPRINT
  // load string onto spool.  Only needed when the robot is being built.
//  loadspool();
  // test suite
//  testKinematics( 0, 0);
//  testKinematics( 5, 0);
//  testKinematics( 0, 5);
//  testKinematics(-5,-5);
//  testFullCircle();
//    testInterpolate2();
//    testAcceleration();
//    testMaxVel();
//  testArcs();
#endif  // SMALL_FOOTPRINT

  // initialize the plotter position.
  teleport(0,0);
  // display the help at startup.
  help();  
}


//------------------------------------------------------------------------------
void loop() {
  // See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
  // listen for serial commands
  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read();
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions
  }
 
  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && buffer[sofar-1]==';') {
    // what if message fails/garbled?
 
    // echo confirmation
    buffer[sofar]=0;
    Serial.println(buffer);
 
    // do something with the command
    processCommand();
 
    // reset the buffer
    sofar=0;
 
    // echo completion
    Serial.print("> ");
  }
}