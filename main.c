#include <stdio.h>
#include <pigpio.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#define TRUE 1
#define FALSE 0
#define PI (3.141592653589793)

#define MPU6050_ADDR 0x68
#define INIT 0x6B
#define I2CBUS 1
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define XACCEL 0x3B
#define YACCEL 0x3D
#define ZACCEL 0x3F
#define TEMP 0x41
#define XGYRO 0x43
#define YGYRO 0x45
#define ZGYRO 0x47

/* These were found by experiment
 * They will differ from chip to chip
 */

#define GYROBIAS_X (15.71)
#define GYROBIAS_Y (-8.5535)
#define GYROBIAS_Z (-20.7775)
#define GYROTEMP_X (-0.6224)
#define GYROTEMP_Y (-0.4553)
#define GYROTEMP_Z (0.21237)

#define ACCELTEMP_X (1.090355)
#define ACCELTEMP_Y (0.19005)
#define ACCELTEMP_Z (-3.055)

#define GYRO_SCALE 2    /* 1000 deg/s */
#define ACCEL_SCALE 2   /* 8g */

#define WIDTH 16        /* width of gyro/accel data */


/*
 * *** CONVENTIONS ***
 *
 * R x_world = x_craft
 * The rotation matrix R converts (relative to world) vectors
 * into (relative to the quadcopter) vectors
 *
 */


/*********************
 * Integer functions *
 *********************/

int twosToInt (unsigned int number, int width)
{
    int bitshift; // represents the power-1 of the last bit
    bitshift = width - 2;

    if (number < 0)
    {
        assert (FALSE);
    }
    else if (number & (2 << bitshift))
    {
        return - (((~number) % (2 << (bitshift + 1))) + 1);
    }
    else
    {
        return number;
    }
}

int signedToInt (unsigned int number, int width)
{
    int bitshift;
    bitshift = width - 2;

    if (number < 0)
    {
        assert (FALSE);
    }
    else if (number & (2 << bitshift))
    {
        return - (number % (2 << bitshift));
    }
    else
    {
        return number;
    }
}


/*********************
 * Useful structures *
 *********************/

typedef struct Quat
{
    double q0, q1, q2, q3;
} Quat;

typedef struct Vector
{
    double x, y, z;
} Vector;

typedef struct AccelData
{
    int x, y, z;
} AccelData;

typedef struct GyroData
{
    int x, y, z;
} GyroData;

typedef struct Accel
{
    float x, y, z;
} Accel;

typedef struct Gyro
{
    float x, y, z;
} Gyro;

/********************
 * Vector Functions *
 ********************/

double magnitude (Vector v)
{
    double norm;
    norm = sqrt (pow (v.x, 2) + pow (v.y, 2) + pow (v.z, 2));
    return norm;
}


Vector normalize (Vector v)
{
    double norm;
    Vector w;
    norm = magnitude (v);
    w.x = v.x / norm;
    w.y = v.y / norm;
    w.z = v.z / norm;

    return w;
}

float quatNorm (Quat q)
{
    return sqrt (pow (q.q0, 2) + pow (q.q1, 2) + pow (q.q2, 2) + pow (q.q3, 2));
}

Quat conjugate (Quat q)
{
    Quat p;
    p.q0 = q.q0;
    p.q1 = - q.q1;
    p.q2 = - q.q2;
    p.q3 = - q.q3;
    return p;
}

Quat inverse (Quat q)
{
    /* the inverse of a quaternion is its conjugate divided by the norm */
    Quat p;
    float norm;
    p = conjugate (q);
    norm = quatNorm (q);
    p.q0 /= norm;
    p.q1 /= norm;
    p.q2 /= norm;
    p.q3 /= norm;
    return p;
}




/**********************
 * Rotation Functions *
 **********************/

Quat axisAngleToQuat(Vector axis, double angle)
{
    /*
     * equation for q found from section 7.3 of
     * "Representing Attitude: Euler Angles, Unit Quaternions,
     * and Rotation Vectors" by James Diebel
     */

    Quat q;

    axis = normalize(axis);
    q.q0 = cos (angle / 2);
    q.q1 = axis.x * sin (angle / 2);
    q.q2 = axis.y * sin (angle / 2);
    q.q3 = axis.z * sin (angle / 2);

    return q;
}

Quat rotVectorToQuat (Vector v)
{
    /* in a rotation vector, the direction is the axis
     * and the magnitude represents the angle
     */

    double norm;
    norm = magnitude (v);

    return axisAngleToQuat(v, norm);
}

Quat gravToQuat (Vector g)
{
    /* we want to find the quaternion that
     * rotates -e_3 into the craft's measured gravity vector, g
     *
     * g cross (0,0,-1) = (-g2, g1, 0) = |g|*sin(angle)*axis
     * we find the axis and angle and then convert with rotVectorToQuat
     */

    Vector crossProd, axis;
    double angle, sinAngle, norm;

    crossProd.x = -g.y;
    crossProd.y = g.x;
    crossProd.z = 0;

    norm = magnitude(g);

    axis.x = crossProd.x / norm;
    axis.y = crossProd.y / norm;
    axis.z = crossProd.z / norm;

    sinAngle = magnitude(axis);
    angle = asin(sinAngle);
    return axisAngleToQuat(axis, angle);
}

/******************
 * Data retrieval *
 ******************/

int getWord (int handle, int reg)
{
    int data, newData;
    int firstByte, secondByte;
    data = i2cReadWordData (handle, reg);
    if (data < 0)
    {
        return -1;
    }
    firstByte = data % 0x100;
    secondByte = data / 0x100;
    newData = firstByte * 0x100 + secondByte;
    return newData;
}

float getTemp (int handle)
{
    int temp;
    float realTemp;
    temp = getWord (handle, TEMP);
    int twos = twosToInt (temp, WIDTH);
    realTemp = (twos / 340.0) + 36.53;

    if (temp == -1)
    {
        return -1;
    }
    else
    {
        return realTemp;
    }
}

AccelData getAccel (int handle)
{
    AccelData a;
    a.x = twosToInt (getWord (handle, XACCEL), WIDTH);
    a.y = twosToInt (getWord (handle, YACCEL), WIDTH);
    a.z = twosToInt (getWord (handle, ZACCEL), WIDTH);
    return a;
}

GyroData getGyro (int handle)
{
    GyroData g;
    g.x = twosToInt (getWord (handle, XGYRO), WIDTH);
    g.y = twosToInt (getWord (handle, YGYRO), WIDTH);
    g.z = twosToInt (getWord (handle, ZGYRO), WIDTH);
    return g;
}


/**************************
 * Accel + Gyro Functions *
 **************************/

Accel bitsToGees (Accel a)
{
    /* convert bits to units of g (9.8 m/s^2)
     * using the accelerometer scale:
     *
     * ACCEL_SCALE = 0 ==> +/- 2g full scale
     *               1 ==> +/- 4g
     *               2 ==> +/- 8g
     *               3 ==> +/- 16g
     *
     * number of bits (size of the "full scale") is #define'd as WIDTH (16 bits)
     * since the MPU6050 uses twos complement, the full scale is +/- 32768
     */

    a.x *= pow (2, 1 + ACCEL_SCALE) / pow (2, WIDTH - 1);
    a.y *= pow (2, 1 + ACCEL_SCALE) / pow (2, WIDTH - 1);
    a.z *= pow (2, 1 + ACCEL_SCALE) / pow (2, WIDTH - 1);

    return a;
}

Gyro bitsToRadsPerSec (Gyro g)
{
    /* convert bits to units of radians per second
     * using the gyroscope scale:
     *
     * Gyro_SCALE = 0 ==> +/- 250 deg/s full scale
     *              1 ==> +/- 500 deg/s
     *              2 ==> +/- 1000 deg/s
     *              3 ==> +/- 2000 deg/s
     *
     * number of bits (size of the "full scale") is #define'd as WIDTH (16 bits)
     * since the MPU6050 uses twos complement, the full scale is +/- 32768
     *
     * 1 degree/second is converted to pi/180 radians/second
     */

    g.x *= 250 / pow (2, (WIDTH - 1 - GYRO_SCALE));
    g.y *= 250 / pow (2, (WIDTH - 1 - GYRO_SCALE));
    g.z *= 250 / pow (2, (WIDTH - 1 - GYRO_SCALE));

    g.x *= (PI / 180);
    g.y *= (PI / 180);
    g.z *= (PI / 180);

    return g;
}

/*
 * Accelerometer and gyroscope are adjusted based on values taken
 * from experiments where the chip was kept still, a hairdryer was used to
 * adjust the temperature, and linear regression was performed
 *
 * The adjusted values are meant to approximate 0°C values
 */

Accel adjustedAccel (AccelData a, float temp)
{
    /* temperature must be provided in celsius */
    Accel output;
    output.x = a.x - temp * ACCELTEMP_X;
    output.y = a.y - temp * ACCELTEMP_Y;
    output.z = a.z - temp * ACCELTEMP_Z;

    output = bitsToGees (output);

    return output;
}

Gyro adjustedGyro (GyroData g, float temp)
{
    /* val = c + dt */

    /* temperature must be provided in celsius */
    Gyro output;
    output.x = g.x - GYROBIAS_X;
    output.y = g.y - GYROBIAS_Y;
    output.z = g.z - GYROBIAS_Z;

    output.x -= temp * GYROTEMP_X;
    output.y -= temp * GYROTEMP_Y;
    output.z -= temp * GYROTEMP_Z;

    /* currently in bits
     * will convert to radians/sec
     */

    output = bitsToRadsPerSec (output);

    return output;
}


/*****************
 * Configuration *
 *****************/

int gyroConfig (int handle)
{
    /*
     *  bits 3 and 4 control sensitivity
     *  0 - 250 deg/s
     *  1 - 500 deg/s
     *  2 - 1000 deg/s
     *  3 - 2000 deg/s
     */
    int val;
    val = GYRO_SCALE << 3;
    return i2cWriteByteData (handle, GYRO_CONFIG, val);
}

int accelConfig (int handle)
{
    /*
     *  bits 3 and 4 control sensitivity
     *  0 - 2g
     *  1 - 4g
     *  2 - 8g
     *  3 - 16g
     */
    int val;
    val = ACCEL_SCALE << 3;
    return i2cWriteByteData (handle, ACCEL_CONFIG, val);
}

/*****************
 * Main function *
 *****************/

int init ()
{
    int handle;
    int sleepPass, gyroPass, accelPass;

    gpioInitialise();
    handle = i2cOpen (I2CBUS, MPU6050_ADDR, 0);

    /* Turn off sleep mode */
    sleepPass = i2cWriteByteData (handle, INIT, 0);

    gyroPass = gyroConfig (handle);
    accelPass = accelConfig (handle);
    if (sleepPass == gyroPass == accelPass == 0)
    {
        /* no failures */
        return handle;
    }

}

Quat initGravity (int handle)
{
    /* returns a quaternion for the gravity vector */

    Accel gravity;
    AccelData gravityRaw;
    Vector gravVector, d;
    float temp;
    Quat q;

    temp = getTemp (handle);
    gravityRaw = getAccel (handle);
    gravity = adjustedAccel (gravityRaw, temp);

    gravVector.x = gravity.x;
    gravVector.y = gravity.y;
    gravVector.z = gravity.z;

    gravVector = normalize (gravVector);
    printf ("g: %f, %f, %f\n", gravVector.x,
            gravVector.y, gravVector.z);
    q = rotVectorToQuat (gravVector);
    q = inverse (q);


    // THIS DOESN'T WORK
    d.x = 2*(q.q1 * q.q3 - q.q0 * q.q2);
    d.y = 2*(q.q0 * q.q1 + q.q2 * q.q3);
    d.z = 2*(pow (q.q0, 2) + pow (q.q3, 2) - 0.5);
    printf ("d: %f, %f, %f\n", d.x, d.y, d.z);


    // THIS WORKS?
    Vector v;
    v.x = q.q1 / sqrt(1 - pow(q.q0,2));
    v.y = q.q2 / sqrt(1 - pow(q.q0,2));
    v.z = q.q3 / sqrt(1 - pow(q.q0,2));
    printf("v: %f, %f, %f\n", v.x, v.y, v.z);


    

    /* TODO:
     * convert gravVector to a unit quaternion
     * via page 17 section 6.12
     *
     * TODO:
     * figure out if I need gravVector or maybe its negative?
     */

    return q;


}

int cleanup (int handle)
{
    i2cClose (handle);
}


int main()
{


    /* initialize and configure */


    int handle;
    handle = init();

//    float temp;
//
//    Quat q;
//    Vector v, w;
//    double norm;
//
//    AccelData accelReadings;
//    Accel a;
//
//    temp = getTemp(handle);
//    accelReadings = getAccel(handle);
//
//    a = adjustedAccel(accelReadings, temp);
//
//    GyroData gD;
//    Gyro g;
//
//    gD = getGyro(handle);
//
//    printf("gD: %d, %d, %d\n", gD.x, gD.y, gD.z);
//    g = adjustedGyro(gD, temp);
//    printf("gD: %d, %d, %d\n", gD.x, gD.y, gD.z);
//    printf("g: %f, %f, %f\n", g.x, g.y, g.z);

    initGravity (handle);

    printf("\n\n\n");


    Vector g;
    g.x = 1;
    g.y = 1;
    g.z = -1;

    Quat q;
    q = gravToQuat(g);
    printf("q: %f, %f, %f, %f\n", q);



    Vector d;
    d.x = 2*(q.q1 * q.q3 - q.q0 * q.q2);
    d.y = 2*(q.q0 * q.q1 + q.q2 * q.q3);
    d.z = 2*(pow (q.q0, 2) + pow (q.q3, 2) - 0.5);
    printf ("d: %f, %f, %f\n", d.x, d.y, d.z);

    /*    while (TRUE)
        {
            temp = getTemp(handle);
            printf ("temp: %f\n", temp);
            GyroData gyroReadings;
            AccelData accelReadings;
            Gyro g;
            gyroReadings = getGyro (handle);
            accelReadings = getAccel (handle);
            g = adjustedGyro(gyroReadings, temp);
            printf ("gyro x %d y %d z %d\nrealgyro x %f y %f z %f\naccel x %d y %d z %d\n", gyroReadings.x, gyroReadings.y, gyroReadings.z, g.z, g.y, g.z, accelReadings.x, accelReadings.y, accelReadings.z);
        }
    */


    /* clean up */
    cleanup (handle);

}






