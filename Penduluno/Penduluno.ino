#include <math.h>

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3                                 // Chip Select goes to Analog 3
#define LCD_CD A2                                 // Command/Data goes to Analog 2
#define LCD_WR A1                                 // LCD Write goes to Analog 1
#define LCD_RD A0                                 // LCD Read goes to Analog 0

#define LCD_RESET A4                              // Can alternately just connect to Arduino's reset pin

#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;

/* Defines */
#define SCREEN_WIDTH  320                         //
#define SCREEN_HEIGHT 240                         // Taille de l'écran

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
#define GREY     0xEEEE

/* =================================== */
/*** Triple Pendulum Simulation with RK4 ***/

///////////// Global variables /////////////
// PI
// double const PI = 3.1415927;
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

// Gravity
const double g = 9.81;

// Masses
double m1;
double m2 ;
double m3 ;

// Lengths for  axes
const double le1 = 50.0;
const double le2 = 50.0;
const double le3 = 50.0;

// Length of axes
// into algorithm
const double l1 = 1.0;
const double l2 = 1.0;
const double l3 = 1.0;

// Position of fix point
const double radius = 7;
const double x0 = 160;
const double y0 = 40;

// Damping coefficient
const double k1=0.1;
const double k2=0.1;
const double k3=0.1;

// angles
double t1;
double t2;
double t3;

// Setps
int steps;

// Trace
#define MAX_TRACE 16
int ti=0;
int tr1[MAX_TRACE][2];
int tr2[MAX_TRACE][2];

void reset()
{
// Masses
    m1 = 1.5 + random(100)/100.0;
    m2 = 1.5 + random(100)/100.0;
    m3 = 2 + random(100)/100.0;

// angles
    t1 = PI/2-random(100)/300.0;
    t2 = PI/2-random(100)/300.0;
    t3 = PI/2-random(100)/300.0;

// Steps
    steps=0;

// Trace 
    ti=0;
    for (int i=0; i<MAX_TRACE;i++) {
        tr1[i][0]=0; tr1[i][1]=0;
        tr2[i][0]=0; tr2[i][1]=0;
    }
}


inline const double F1(double t1,double dt1,double  t2,double  dt2,double  t3,double  dt3,double  g,double l1,double l2,double l3,double m1,double  m2, double m3, double k1, double k2, double k3)
{
    return -(2 * g * m2 * m2 * sin(t1) + 2 * g * m2 * m2 * sin(t1 - 2 * t2) + 4 * dt1 * k1 * l1 * m2 + 2 * dt1 * k1 * l1 * m3 + 2 * dt1 * k2 * l1 * m2 + dt1 * k2 * l1 * m3 + dt1 * k3 * l1 * m2 + 4 * dt2 * dt2 * l2 * m2 * m2 * sin(t1 - t2) + 4 * g * m1 * m2 * sin(t1) + 2 * g * m1 * m3 * sin(t1) + 2 * g * m2 * m3 * sin(t1) - g * m1 * m3 * sin(t1 - 2 * t2 + 2 * t3) - g * m1 * m3 * sin(t1 + 2 * t2 - 2 * t3) + 2 * g * m2 * m3 * sin(t1 - 2 * t2) +
        2 * dt1 * dt1 * l1 * m2 * m2 * sin(2 * t1 - 2 * t2) - 2 * dt1 * k2 * l1 * m2 * cos(2 * t1 - 2 * t2) - dt1 * k2 * l1 * m3 * cos(2 * t1 - 2 * t2) - dt1 * k3 * l1 * m2 * cos(2 * t1 - 2 * t2) - 2 * dt1 * k1 * l1 * m3 * cos(2 * t2 - 2 * t3) + dt1 * k2 * l1 * m3 * cos(2 * t1 - 2 * t3) - dt1 * k3 * l1 * m2 * cos(2 * t1 - 2 * t3) - dt1 * k2 * l1 * m3 * cos(2 * t2 - 2 * t3) + dt1 * k3 * l1 * m2 * cos(2 * t2 - 2 * t3) +
        4 * dt2 * dt2 * l2 * m2 * m3 * sin(t1 - t2) + 2 * dt3 * dt3 * l3 * m2 * m3 * sin(t1 - t3) + dt2 * k2 * l2 * m3 * cos(t1 + t2 - 2 * t3) - dt2 * k3 * l2 * m2 * cos(t1 + t2 - 2 * t3) +
        2 * dt1 * dt1 * l1 * m2 * m3 * sin(2 * t1 - 2 * t2) - dt2 * k2 * l2 * m3 * cos(t1 - 3 * t2 + 2 * t3) + dt2 * k3 * l2 * m2 * cos(t1 - 3 * t2 + 2 * t3) + 2 * dt3 * dt3 * l3 * m2 * m3 * sin(t1 - 2 * t2 + t3)) / (l1 * (4 * m1 * m2 + 2 * m1 * m3 + 2 * m2 * m3 - 2 * m2 * m2 * cos(2 * t1 - 2 * t2) + 2 * m2 * m2 - 2 * m2 * m3 * cos(2 * t1 - 2 * t2) - 2 * m1 * m3 * cos(2 * t2 - 2 * t3)));
}


inline const double F2(double t1,double  dt1,double  t2,double  dt2,double  t3,double  dt3,double  g,double  l1,double  l2,double  l3,double  m1,double  m2,double  m3,double  k1,double  k2,double  k3)
{
    return (2 * g * m2 * m2 * sin(2 * t1 - t2) - 2 * g * m2 * m2 * sin(t2) - 4 * dt2 * k2 * l2 * m1 - 2 * dt2 * k2 * l2 * m2 - 2 * dt2 * k3 * l2 * m1 - dt2 * k2 * l2 * m3 - dt2 * k3 * l2 * m2 + 2 * g * m1 * m2 * sin(2 * t1 - t2) +
        g * m1 * m3 * sin(2 * t1 - t2) + 2 * g * m2 * m3 * sin(2 * t1 - t2) + 4 * dt1 * dt1 * l1 * m2 * m2 * sin(t1 - t2) - 2 * g * m1 * m2 * sin(t2) - g * m1 * m3 * sin(t2) - 2 * g * m2 * m3 * sin(t2) -
        g * m1 * m3 * sin(2 * t1 + t2 - 2 * t3) - g * m1 * m3 * sin(t2 - 2 * t3) + 2 * dt2 * dt2 * l2 * m2 * m2 * sin(2 * t1 - 2 * t2) + 4 * dt1 * k1 * l1 * m2 * cos(t1 - t2) - 4 * dt1 * k2 * l1 * m1 * cos(t1 - t2) +
        2 * dt1 * k1 * l1 * m3 * cos(t1 - t2) - 2 * dt1 * k3 * l1 * m1 * cos(t1 - t2) + 2 * dt2 * k2 * l2 * m2 * cos(2 * t1 - 2 * t2) + dt2 * k2 * l2 * m3 * cos(2 * t1 - 2 * t2) + dt2 * k3 * l2 * m2 * cos(2 * t1 - 2 * t2) +
        dt2 * k2 * l2 * m3 * cos(2 * t1 - 2 * t3) + 2 * dt2 * k3 * l2 * m1 * cos(2 * t2 - 2 * t3) - dt2 * k3 * l2 * m2 * cos(2 * t1 - 2 * t3) - dt2 * k2 * l2 * m3 * cos(2 * t2 - 2 * t3) + dt2 * k3 * l2 * m2 * cos(2 * t2 - 2 * t3) + 4 * dt1 * dt1 * l1 * m1 * m2 * sin(t1 - t2) + 2 * dt1 * dt1 * l1 * m1 * m3 * sin(t1 - t2) + 4 * dt1 * dt1 * l1 * m2 * m3 * sin(t1 - t2) - 4 * dt3 * dt3 * l3 * m1 * m3 * sin(t2 - t3) -
        2 * dt3 * dt3 * l3 * m2 * m3 * sin(t2 - t3) - 2 * dt1 * k1 * l1 * m3 * cos(t1 + t2 - 2 * t3) + 2 * dt1 * k3 * l1 * m1 * cos(t1 + t2 - 2 * t3) - dt1 * k2 * l1 * m3 * cos(t1 + t2 - 2 * t3) +
        dt1 * k3 * l1 * m2 * cos(t1 + t2 - 2 * t3) + 2 * dt3 * dt3 * l3 * m2 * m3 * sin(2 * t1 - t2 - t3) + 2 * dt2 * dt2 * l2 * m2 * m3 * sin(2 * t1 - 2 * t2) - 2 * dt2 * dt2 * l2 * m1 * m3 * sin(2 * t2 - 2 * t3) +
        dt1 * k2 * l1 * m3 * cos(t2 - 3 * t1 + 2 * t3) - dt1 * k3 * l1 * m2 * cos(t2 - 3 * t1 + 2 * t3) - 2 * dt1 * dt1 * l1 * m1 * m3 * sin(t1 + t2 - 2 * t3)) / (2 * l2 * (2 * m1 * m2 + m1 * m3 + m2 * m3 - m2 * m2 * cos(2 * t1 -
        2 * t2) + m2 * m2 - m2 * m3 * cos(2 * t1 - 2 * t2) - m1 * m3 * cos(2 * t2 - 2 * t3)));
}


inline const double F3(double t1,double  dt1,double  t2,double  dt2,double  t3,double  dt3,double  g,double  l1,double  l2,double  l3,double  m1,double m2,double  m3,double  k1,double  k2,double  k3)
{
    return (g * m1 * m3 * m3 * sin(2 * t1 - t3) - 2 * dt3 * k3 * l3 * m2 * m2 + g * m1 * m3 * m3 * sin(2 * t2 - t3) - g * m1 * m3 * m3 * sin(t3) - g * m1 * m3 * m3 * sin(2 * t1 - 2 * t2 + t3) +
        2 * dt3 * dt3 * l3 * m1 * m3 * m3 * sin(2 * t2 - 2 * t3) - dt2 * k2 * l2 * m3 * m3 * cos(2 * t1 - 3 * t2 + t3) + dt2 * k3 * l2 * m2 * m2 * cos(2 * t1 - 3 * t2 + t3) - 2 * dt1 * dt1 * l1 * m1 * m3 * m3 * sin(t1 - 2 * t2 + t3) +
        2 * dt1 * k1 * l1 * m3 * m3 * cos(t1 - t3) + 2 * dt1 * k2 * l1 * m3 * m3 * cos(t1 - t3) - 2 * dt1 * k3 * l1 * m2 * m2 * cos(t1 - t3) + 2 * dt2 * k2 * l2 * m3 * m3 * cos(t2 - t3) - 2 * dt2 * k3 * l2 * m2 * m2 * cos(t2 -
        t3) - 4 * dt3 * k3 * l3 * m1 * m2 - 2 * dt3 * k3 * l3 * m1 * m3 - 2 * dt3 * k3 * l3 * m2 * m3 + g * m1 * m2 * m3 * sin(2 * t1 - t3) + g * m1 * m2 * m3 * sin(2 * t2 - t3) - dt1 * k2 * l1 * m3 * m3 * cos(3 * t1 - 2 * t2 - t3) +
        dt1 * k3 * l1 * m2 * m2 * cos(3 * t1 - 2 * t2 - t3) - dt2 * k2 * l2 * m3 * m3 * cos(2 * t1 - t2 - t3) + dt2 * k3 * l2 * m2 * m2 * cos(2 * t1 - t2 - t3) + 2 * dt3 * k3 * l3 * m2 * m2 * cos(2 * t1 - 2 * t2) +
        2 * dt1 * dt1 * l1 * m1 * m3 * m3 * sin(t1 - t3) + 4 * dt2 * dt2 * l2 * m1 * m3 * m3 * sin(t2 - t3) - 2 * dt1 * k1 * l1 * m3 * m3 * cos(t1 - 2 * t2 + t3) - dt1 * k2 * l1 * m3 * m3 * cos(t1 - 2 * t2 + t3) +
        dt1 * k3 * l1 * m2 * m2 * cos(t1 - 2 * t2 + t3) - g * m1 * m2 * m3 * sin(t3) - g * m1 * m2 * m3 * sin(2 * t1 - 2 * t2 + t3) - dt2 * k2 * l2 * m2 * m3 * cos(2 * t1 - 3 * t2 + t3) + dt2 * k3 * l2 * m2 * m3 * cos(2 * t1 - 3 * t2 + t3) - 2 * dt1 * dt1 * l1 * m1 * m2 * m3 * sin(t1 - 2 * t2 + t3) + 2 * dt1 * k1 * l1 * m2 * m3 * cos(t1 - t3) + 2 * dt1 * k2 * l1 * m1 * m3 * cos(t1 - t3) - 4 * dt1 * k3 * l1 * m1 * m2 * cos(t1 - t3) +
        2 * dt1 * k2 * l1 * m2 * m3 * cos(t1 - t3) - 2 * dt1 * k3 * l1 * m1 * m3 * cos(t1 - t3) - 2 * dt1 * k3 * l1 * m2 * m3 * cos(t1 - t3) + 4 * dt2 * k2 * l2 * m1 * m3 * cos(t2 - t3) - 4 * dt2 * k3 * l2 * m1 * m2 * cos(t2 -
        t3) + 2 * dt2 * k2 * l2 * m2 * m3 * cos(t2 - t3) - 2 * dt2 * k3 * l2 * m2 * m3 * cos(t2 - t3) - dt1 * k2 * l1 * m2 * m3 * cos(3 * t1 - 2 * t2 - t3) - dt2 * k2 * l2 * m2 * m3 * cos(2 * t1 - t2 - t3) +
        dt1 * k3 * l1 * m2 * m3 * cos(3 * t1 - 2 * t2 - t3) + dt2 * k3 * l2 * m2 * m3 * cos(2 * t1 - t2 - t3) + 2 * dt3 * k3 * l3 * m2 * m3 * cos(2 * t1 - 2 * t2) + 2 * dt3 * k3 * l3 * m1 * m3 * cos(2 * t2 - 2 * t3) +
        2 * dt1 * dt1 * l1 * m1 * m2 * m3 * sin(t1 - t3) + 4 * dt2 * dt2 * l2 * m1 * m2 * m3 * sin(t2 - t3) - 2 * dt1 * k1 * l1 * m2 * m3 * cos(t1 - 2 * t2 + t3) + 2 * dt1 * k2 * l1 * m1 * m3 * cos(t1 - 2 * t2 + t3) -
        dt1 * k2 * l1 * m2 * m3 * cos(t1 - 2 * t2 + t3) + 2 * dt1 * k3 * l1 * m1 * m3 * cos(t1 - 2 * t2 + t3) + dt1 * k3 * l1 * m2 * m3 * cos(t1 - 2 * t2 + t3)) / (2 * l3 * m3 * (2 * m1 * m2 + m1 * m3 + m2 * m3 - m2 *
        m2 * cos(2 * t1 - 2 * t2) + m2 * m2 - m2 * m3 * cos(2 * t1 - 2 * t2) - m1 * m3 * cos(2 * t2 - 2 * t3)));
}


// delta
double dt1 = 0.0;
double dt2 = 0.0;
double dt3 = 0.0;

// Time step
const double h = 0.02;

void ProcessNext()
{
// RK4 coefficients
    double k[4][6];

    k[0][0] = h * dt1;
    k[0][1] = h * F1(t1, dt1, t2, dt2, t3, dt3, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[0][2] = h * dt2;
    k[0][3] = h * F2(t1, dt1, t2, dt2, t3, dt3, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[0][4] = h * dt3;
    k[0][5] = h * F3(t1, dt1, t2, dt2, t3, dt3, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

    k[1][0] = h * (dt1 + k[0][1] / 2);
    k[1][1] = h * F1(t1 + k[0][0] / 2, dt1 + k[0][1] / 2, t2 + k[0][2] / 2, dt2 + k[0][3] / 2, t3 + k[0][4] / 2, dt3 + k[0][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[1][2] = h * (dt2 + k[0][3] / 2);
    k[1][3] = h * F2(t1 + k[0][0] / 2, dt1 + k[0][1] / 2, t2 + k[0][2] / 2, dt2 + k[0][3] / 2, t3 + k[0][4] / 2, dt3 + k[0][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[1][4] = h * (dt3 + k[0][5] / 2);
    k[1][5] = h * F3(t1 + k[0][0] / 2, dt1 + k[0][1] / 2, t2 + k[0][2] / 2, dt2 + k[0][3] / 2, t3 + k[0][4] / 2, dt3 + k[0][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

    k[2][0] = h * (dt1 + k[1][1] / 2);
    k[2][1] = h * F1(t1 + k[1][0] / 2, dt1 + k[1][1] / 2, t2 + k[1][2] / 2, dt2 + k[1][3] / 2, t3 + k[1][4] / 2, dt3 + k[1][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[2][2] = h * (dt2 + k[1][3] / 2);
    k[2][3] = h * F2(t1 + k[1][0] / 2, dt1 + k[1][1] / 2, t2 + k[1][2] / 2, dt2 + k[1][3] / 2, t3 + k[1][4] / 2, dt3 + k[1][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[2][4] = h * (dt3 + k[1][5] / 2);
    k[2][5] = h * F3(t1 + k[1][0] / 2, dt1 + k[1][1] / 2, t2 + k[1][2] / 2, dt2 + k[1][3] / 2, t3 + k[1][4] / 2, dt3 + k[1][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

    k[3][0] = h * (dt1 + k[2][1]);
    k[3][1] = h * F1(t1 + k[2][0], dt1 + k[2][1], t2 + k[2][2], dt2 + k[2][3], t3 + k[2][4], dt3 + k[2][5], g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[3][2] = h * (dt2 + k[2][3]);
    k[3][3] = h * F2(t1 + k[2][0], dt1 + k[2][1], t2 + k[2][2], dt2 + k[2][3], t3 + k[2][4], dt3 + k[2][5], g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
    k[3][4] = h * (dt3 + k[2][5]);
    k[3][5] = h * F3(t1 + k[2][0], dt1 + k[2][1], t2 + k[2][2], dt2 + k[2][3], t3 + k[2][4], dt3 + k[2][5], g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

// Save first computed variables
    double t1_1 = t1;
    double dt1_1 = dt1;
    double t2_1 = t2;
    double dt2_1 = dt2;
    double t3_1 = t3;
    double dt3_1 = dt3;

    double t1_2 = t1;
    double dt1_2 = dt1;
    double t2_2 = t2;
    double dt2_2 = dt2;
    double t3_2 = t3;
    double dt3_2 = dt3;

// Compute Next positions
    t1 = t1 + (k[0][0] + 2 * k[1][0] + 2 * k[2][0] + k[3][0]) / 6;
    dt1 = dt1 + (k[0][1] + 2 * k[1][1] + 2 * k[2][1] + k[3][1]) / 6;
    t2 = t2 + (k[0][2] + 2 * k[1][2] + 2 * k[2][2] + k[3][2]) / 6;
    dt2 = dt2 + (k[0][3] + 2 * k[1][3] + 2 * k[2][3] + k[3][3]) / 6;
    t3 = t3 + (k[0][4] + 2 * k[1][4] + 2 * k[2][4] + k[3][4]) / 6;
    dt3 = dt3 + (k[0][5] + 2 * k[1][5] + 2 * k[2][5] + k[3][5]) / 6;

// Loop for RK4 with adaptative step
    for (unsigned int i = 0; i < 2; i++)
    {

        k[0][0] = h / 2 * dt1_1;
        k[0][1] = h / 2 * F1(t1_1, dt1_1, t2_1, dt2_1, t3_1, dt3_1, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[0][2] = h / 2 * dt2_1;
        k[0][3] = h / 2 * F2(t1_1, dt1_1, t2_1, dt2_1, t3_1, dt3_1, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[0][4] = h / 2 * dt3_1;
        k[0][5] = h / 2 * F3(t1_1, dt1_1, t2_1, dt2_1, t3_1, dt3_1, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

        k[1][0] = h / 2 * (dt1_1 + k[0][1] / 2);
        k[1][1] = h / 2 * F1(t1_1 + k[0][0] / 2, dt1_1 + k[0][1] / 2, t2_1 + k[0][2] / 2, dt2_1 + k[0][3] / 2, t3_1 + k[0][4] / 2, dt3_1 + k[0][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[1][2] = h / 2 * (dt2_1 + k[0][3] / 2);  tr1[i][0]=0;

        k[1][3] = h / 2 * F2(t1_1 + k[0][0] / 2, dt1_1 + k[0][1] / 2, t2_1 + k[0][2] / 2, dt2_1 + k[0][3] / 2, t3_1 + k[0][4] / 2, dt3_1 + k[0][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[1][4] = h / 2 * (dt3_1 + k[0][5] / 2);
        k[1][5] = h / 2 * F3(t1_1 + k[0][0] / 2, dt1_1 + k[0][1] / 2, t2_1 + k[0][2] / 2, dt2_1 + k[0][3] / 2, t3_1 + k[0][4] / 2, dt3_1 + k[0][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

        k[2][0] = h / 2 * (dt1_1 + k[1][1] / 2);
        k[2][1] = h / 2 * F1(t1_1 + k[1][0] / 2, dt1_1 + k[1][1] / 2, t2_1 + k[1][2] / 2, dt2_1 + k[1][3] / 2, t3_1 + k[1][4] / 2, dt3_1 + k[1][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[2][2] = h / 2 * (dt2_1 + k[1][3] / 2);
        k[2][3] = h / 2 * F2(t1_1 + k[1][0] / 2, dt1_1 + k[1][1] / 2, t2_1 + k[1][2] / 2, dt2_1 + k[1][3] / 2, t3_1 + k[1][4] / 2, dt3_1 + k[1][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[2][4] = h / 2 * (dt3_1 + k[1][5] / 2);
        k[2][5] = h / 2 * F3(t1_1 + k[1][0] / 2, dt1_1 + k[1][1] / 2, t2_1 + k[1][2] / 2, dt2_1 + k[1][3] / 2, t3_1 + k[1][4] / 2, dt3_1 + k[1][5] / 2, g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

        k[3][0] = h / 2 * (dt1_1 + k[2][1]);
        k[3][1] = h / 2 * F1(t1_1 + k[2][0], dt1_1 + k[2][1], t2_1 + k[2][2], dt2_1 + k[2][3], t3_1 + k[2][4], dt3_1 + k[2][5], g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[3][2] = h / 2 * (dt2_1 + k[2][3]);
        k[3][3] = h / 2 * F2(t1_1 + k[2][0], dt1_1 + k[2][1], t2_1 + k[2][2], dt2_1 + k[2][3], t3_1 + k[2][4], dt3_1 + k[2][5], g, l1, l2, l3, m1, m2, m3, k1, k2, k3);
        k[3][4] = h / 2 * (dt3_1 + k[2][5]);
        k[3][5] = h / 2 * F3(t1_1 + k[2][0], dt1_1 + k[2][1], t2_1 + k[2][2], dt2_1 + k[2][3], t3_1 + k[2][4], dt3_1 + k[2][5], g, l1, l2, l3, m1, m2, m3, k1, k2, k3);

// Compute Next positions
        t1_1 = t1_1 + (k[0][0] + 2 * k[1][0] + 2 * k[2][0] + k[3][0]) / 6;
        dt1_1 = dt1_1 + (k[0][1] + 2 * k[1][1] + 2 * k[2][1] + k[3][1]) / 6;
        t2_1 = t2_1 + (k[0][2] + 2 * k[1][2] + 2 * k[2][2] + k[3][2]) / 6;
        dt2_1 = dt2_1 + (k[0][3] + 2 * k[1][3] + 2 * k[2][3] + k[3][3]) / 6;
        t3_1 = t3_1 + (k[0][4] + 2 * k[1][4] + 2 * k[2][4] + k[3][4]) / 6;
        dt3_1 = dt3_1 + (k[0][5] + 2 * k[1][5] + 2 * k[2][5] + k[3][5]) / 6;

    }
// Angles modulo 2*PI
    t1 = fmod(t1,(2 * PI));
    t2 = fmod(t2,(2 * PI));
    t3 = fmod(t3,(2 * PI));

}


/* Fonction setup */
void setup()
{
    reset();
/* Initialise l'écran LCD */
    tft.reset();
    tft.begin(tft.readID());
    tft.fillScreen(GREY);
}



uint16_t x1, x2, x3, y1, y2, y3;

/* Fonction loop() */
void loop()
{
    if (steps++>2048)
    {
        reset();
        tft.fillScreen(GREY);
    }

    ProcessNext();
// Clear axes
    tft.drawLine(y0,x0,y1,x1, GREY);
    tft.drawLine(y1,x1,y2,x2, GREY);
    tft.drawLine(y2,x2,y3,x3, GREY);

// Clear masses
    // tft.fillCircle(y0,x0,5, GREY);
    tft.fillCircle(y1,x1,5, GREY);
    tft.fillCircle(y2,x2,5, GREY);
    tft.fillCircle(y3,x3,5, GREY);

// Coordinates of circles
    x1 = x0 + le1 * sin(t1);
    y1 = y0 + le1 * cos(t1);
    x2 = x1 + le2 * sin(t2);
    y2 = y1 + le2 * cos(t2);
    x3 = x2 + le3 * sin(t3);
    y3 = y2 + le3 * cos(t3);

// Draw axes
    tft.drawLine(y0,x0,y1,x1, BLACK);
    tft.drawLine(y1,x1,y2,x2, BLACK);
    tft.drawLine(y2,x2,y3,x3, BLACK);

// Draw masses
    tft.fillCircle(y0,x0,5, BLACK);
    tft.fillCircle(y1,x1,5, BLACK);
    tft.fillCircle(y2,x2,5, BLACK);
    tft.fillCircle(y3,x3,5, BLACK);

// Draw trace
    for (int i=0; i<MAX_TRACE;i++)
    {
        tft.drawPixel(tr1[i][1],tr1[i][0],BLUE);
        tft.drawPixel(tr2[i][1],tr2[i][0],RED);
    }

// Save last point
    tr1[ti%MAX_TRACE][0]=x2;
    tr1[ti%MAX_TRACE][1]=y2;
    tr2[ti%MAX_TRACE][0]=x3;
    tr2[ti%MAX_TRACE][1]=y3;
    ti++;

}
