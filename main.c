#include <stdio.h>
#include <pigpio.h>
#include <math.h>
/*
 *  main.c
 *  pi-quad
 *  Raspberry Pi based quadcopter flight controller
 *  Uses the MPU-6050 for temperature, accelerometer,
 *    and gyroscope measurements
 *
 *  Real-time programming tidbits from the preempt rt wiki
 */


#include <assert.h>
#include <unistd.h>
#include <errno.h>

#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>

/* shared memory */
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

/* Real-time #defines */
#define MY_PRIORITY (80)    /* above kernel RT */
#define MAX_SAFE_STACK (8*1024) /* max stack size which is guaranteed
                                   safe to access without faulting */
#define NSEC_PER_SEC (1000000000) /* number of nanoseconds per second */


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

#define K_I (0)
#define K_P (5)


/*
 * *** CONVENTIONS ***
 *
 * R x_world = x_craft
 * The rotation matrix R converts (relative to world) vectors
 * into (relative to the quadcopter) vectors
 *
 */

/*********************
 * Useful structures *
 *********************/

typedef struct JoystickData
{
    int axis;
    int value;
} JoystickData;

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


/*****************
 * Shared Memory *
 *****************/

int allocateMem()
{
    int shmid;
    key_t key;

    key = ftok("/home/jackson/quadcopter/.ipc/ipc_file", 'R');
    if (key == -1)
    {
        perror("ftok");
    }
    shmid = shmget(key, sizeof(JoystickData), IPC_CREAT|IPC_EXCL|0600);
    if (shmid == -1)
    {
        printf("Shared memory seg exists - opening as client\n");
        shmid = shmget(key, sizeof(JoystickData), 0);
        if (shmid == -1)
        {
            perror("shmget");
            return;
        }
    }

    return shmid;
}

JoystickData * attachMem(int id)
{
    JoystickData * jPtr;

    if ((jPtr = (JoystickData *)shmat(id, NULL, 0)) == (JoystickData *)-1)
    {
        perror("shmat");
        return;
    }
    return jPtr;
}

void detachMem(JoystickData * jPtr)
{
    if (shmdt(jPtr) == -1)
    {
        perror("shmdt");
    }
}

void deallocateMem(int id)
{
    if (shmctl(id, IPC_RMID, NULL) == -1)
    {
        perror("shmctl");
    }
}




/***********************
 * Real-time functions *
 ***********************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}


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

Vector cross (Vector u, Vector v)
{
    Vector w;
    w.x = u.y * v.z - u.z * v.y;
    w.y = u.z * v.x - u.x * v.z;
    w.z = u.x * v.y - u.y * v.x;
    return w;
}

Vector scalarMult(Vector v, double s)
{
    v.x *= s;
    v.y *= s;
    v.z *= s;
    return v;
}

Vector VectorAdd(Vector v, Vector u)
{
    Vector w;
    w.x = v.x + u.x;
    w.y = v.y + u.y;
    w.z = v.z + u.z;
    return w;
}

float quatNorm (Quat q)
{
    return sqrt (pow (q.q0, 2) + pow (q.q1, 2) + pow (q.q2, 2) + pow (q.q3, 2));
}

Quat quatNormalize (Quat q)
{
    double norm;
    norm = quatNorm (q);
    q.q0 /= norm;
    q.q1 /= norm;
    q.q2 /= norm;
    q.q3 /= norm;
    return q;
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
    double dotProd;

    crossProd.x = -g.y;
    crossProd.y = g.x;
    crossProd.z = 0;

    norm = magnitude(g);

    axis.x = crossProd.x / norm;
    axis.y = crossProd.y / norm;
    axis.z = crossProd.z / norm;

    sinAngle = magnitude(axis);
    angle = asin(sinAngle);

    /* Our angle might be off by pi/2 - x
     * We determine this by the sign of the dot product
     */
    
    dotProd = -g.z;
    
    if (dotProd > 0)
    {
        angle = PI - angle;
    }
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

Vector bitsToGees (Vector a)
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

Vector bitsToRadsPerSec (Vector g)
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

Vector adjustedAccel (AccelData a, float temp)
{
    /* temperature must be provided in celsius */
    Vector output;
    output.x = a.x - temp * ACCELTEMP_X;
    output.y = a.y - temp * ACCELTEMP_Y;
    output.z = a.z - temp * ACCELTEMP_Z;

    output = bitsToGees (output);

    return output;
}

Vector adjustedGyro (GyroData g, float temp)
{
    /* val = c + dt */

    /* temperature must be provided in celsius */
    Vector output;
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
 * Config + Init *
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
    /* returns a quaternion that represents
     * rotating -e3 into the gravity vector
     */

    AccelData gravityRaw;
    Vector gravity, d;
    float temp;
    Quat q;

    temp = getTemp (handle);
    gravityRaw = getAccel (handle);
    gravity = adjustedAccel (gravityRaw, temp);
    
    return gravToQuat(gravity);
}

int cleanup (int handle)
{
    i2cClose (handle);
}


/*****************
 * Main function *
 *****************/

int main()
{
    /* joystick setup */
    JoystickData * joyPtr;
    int shmid;

    shmid = allocateMem();
    joyPtr = attachMem(shmid);


    char sendline[1000];
    while (fgets(sendline, 10000, stdin) != NULL)
    {
        printf("%d: %d\n", joyPtr->axis, joyPtr->value);
        sscanf(sendline, "%d: %d", &(joyPtr->axis), &(joyPtr->value));
    }

    detachMem(joyPtr);

}


int NOTmain()
{

    /***** Real-time setup *****/

    struct timespec t;
    struct sched_param param;
    /* reading 36 bits takes 360 us (clock is 100 kHz)
     * 16 for gyro + 16 for accel + 4 error checking
     * Give ourselves an extra 640 us for computation
     * Total: 1 ms
     *
     * This can be changed!
     */
    int interval = 1000000; /* 1 ms = 1000 us */

    /* declare self as real time task */

    param.sched_priority = MY_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
        exit(-1);
    }

    /* lock memory*/

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        exit(-2);
    }

    /* pre-fault stack */

    stack_prefault();
    clock_gettime(CLOCK_MONOTONIC, &t);
    
    /* start after one second */

    t.tv_sec++;


    /* initialize and configure */

    int handle;
    handle = init();

    Vector d, e; // grav, error
    Vector a, w; // accel, ang. vel
    Vector I, wpr; // integral, new ang. vel
    Quat q;

    AccelData accel;
    GyroData gyro;

    float temp;
    double dt, halfdt;
    dt = 0.001;
    halfdt = 0.0005;

    q = initGravity(handle);

    /* main loop */

    while (TRUE)
    {
        /* wait until next shot */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

        /* calculate q from gyro and accel input */

        // Step 1: Measure w and a

        temp = getTemp(handle);
        accel = getAccel(handle);
        gyro = getGyro(handle);

        a = adjustedAccel(accel, temp);
        w = adjustedGyro(gyro, temp);

        // Step 2: Normalize a

        a = normalize(a);

        // Step 3: Get estimated gravity vector d from quat q

        d.x = 2*(q.q0*q.q2 - q.q1*q.q3);
        d.y = 2*(q.q2*q.q3 - q.q0*q.q1);
        d.z = 2*(pow (q.q0, 2) + pow (q.q3, 2) - 0.5);

        // Step 4: Calculate error vector e
        // (Cross between estimated and measured gravity)

        e = cross(a, d);

        // Step 5: Calculate I term
        
        I = VectorAdd(I, scalarMult(e, K_I * dt));

        // Step 6: Calculate P term

        wpr = VectorAdd(w, VectorAdd(scalarMult(e, K_P), I));

        // Step 7: Integrate rate of change with qdot = 0.5q tensor wpr
        
        halfdt = 0.5 * dt;
        q.q0 += halfdt * (-q.q1 * wpr.x - q.q2 * wpr.y - q.q3 * wpr.z);
        q.q1 += halfdt * (q.q0 * wpr.x + q.q2 * wpr.z - q.q3 * wpr.y);
        q.q2 += halfdt * (q.q0 * wpr.y - q.q1 * wpr.z + q.q3 * wpr.x);
        q.q3 += halfdt * (q.q0 * wpr.z + q.q0 * wpr.y - q.q2 * wpr.x);

        q = quatNormalize(q);
        printf("q: %f, %f, %f, %f\n", q.q0, q.q1, q.q2, q.q3);
       // printf("%d\n", t.tv_nsec);

        /* calculate next shot */
        t.tv_nsec += interval;
        while (t.tv_nsec >= NSEC_PER_SEC)
        {
            t.tv_nsec -= NSEC_PER_SEC;
            t.tv_sec++;
        }
    }


    /* clean up */
    cleanup (handle);

}






