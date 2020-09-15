#include "mbed.h"
#include "LSM9DS1.h"


#include "LSM9DS1.h"
 
//DigitalOut myled(LED1);
#define PI 3.14 
#define printff_CALCULATED
#define printff_SPEED 250 
#define DECLINATION -8.58
 
 
#define dt 1/975  
#define gey_sensitivity 8.75 
 
 
void printfGyro();
void printfAccel();
void printfMag();
void printfAttitude(float ax, float ay, float az, float mx, float my, float mz);
 
LSM9DS1 lol(PF_0, PF_1,0xD6, 0x3C);
int main() {
    
    lol.begin();
    if (!lol.begin()) {
        printf("Failed to communicate with LSM9DS1.\n");
        while(1) ; 
    }
    lol.calibrate();
    lol.calibrateMag() ;
    while(1) {
        
        //lol.readTemp();
        
        if ( lol.magAvailable() )
        {
        lol.readMag();
        }
        if ( lol.accelAvailable() )
        {
        lol.readAccel();
        }
        if ( lol.gyroAvailable() )
        {
        lol.readGyro();
        }
        
        printfGyro();  // printfff "G: gx, gy, gz"
        pc.printf("\n") ;
        printfAccel(); // printfff "A: ax, ay, az"
        pc.printf("\n") ;
        printfMag();   // printfff "M: mx, my, mz"
        pc.printf("\n") ;
        // printff the heading and orientation fofun!
        // Call printfff attitude. The LSM9DS1's mag x and y
        // axes are opposite to the accelerometer, so my, mx are
        // substituted for each other.
        printfAttitude(lol.ax, lol.ay, lol.az,lol.gx, lol.gy, lol.gz);
        pc.printf("\n") ;
        wait(1);
   } 
   }              
        
        void printfGyro()
     {
  // Now we can use the gx, gy, and gz variables as we please.
  // Either printfff them as raw ADC values, or calculated in DPS.
        pc.printf("G: ");
        #ifdef printff_CALCULATED 
  // If you want to printffff calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  pc.printf("%f" , lol.calcGyro(lol.gx), 2);
  pc.printf(", ");
  pc.printf("%f" , lol.calcGyro(lol.gy), 2);
  pc.printf(", ");
  pc.printf("%f" , lol.calcGyro(lol.gz), 2);
  pc.printf(" deg/s ");
  pc.printf("\n") ;
/*#elif defined printfff_RAW
  pc.printf(lol.gx);
  pc.printf(", ");
  pc.printf(lol.gy);
  pc.printf(", ");
  pc.printfln(lol.gz);*/
    #endif
}
 
void printfAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either printfff them as raw ADC values, or calculated in g's.
  pc.printf("A: ");
#ifdef printff_CALCULATED
  // If you want to printffff calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  pc.printf("%f" , lol.calcAccel(lol.ax), 2);
  pc.printf(", ");
  pc.printf("%f" , lol.calcAccel(lol.ay), 2);
  pc.printf(", ");
  pc.printf("%f" , lol.calcAccel(lol.az), 2);
  pc.printf(" g");
/*#elif defined printfff_RAW
  pc.printf(lol.ax);
  pc.printf(", ");
  pc.printf(lol.ay);
  pc.printf(", ");
  pc.printfln(lol.az);*/
  
#endif
 
}
 
void printfMag()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either printfff them as raw ADC values, or calculated in Gauss.
  pc.printf("M: ");
#ifdef printff_CALCULATED
  // If you want to printffff calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  pc.printf("%f" , lol.calcMag(lol.mx), 2);
  pc.printf(", ");
  pc.printf("%f" , lol.calcMag(lol.my), 2);
  pc.printf(", ");
  pc.printf("%f" , lol.calcMag(lol.mz), 2);
  pc.printf(" gauss");
  
/*#elif defined printfff_RAW
  pc.printf(lol.mx);
  pc.printf(", ");
  pc.printf(lol.my);
  pc.printf(", ");
  pc.printfln(lol.mz);*/
    #endif
}
 
void printfAttitude(float ax, float ay, float az, float gx, float gy, float gz)
{
  //double  roll = atan2(ay, az);
  //double  pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
    
    double *pitch , *roll ;
    
     *pitch +=((float)gx /gey_sensitivity)*dt ; 
     *roll  -= ((float)gy /gey_sensitivity)*dt ; 
 
    double pitchacc = atan2(ay,az)*180/PI ;
    *pitch = *pitch *0,98+pitchacc*0.02 ; 
    
    double rollacc =atan2(ax,az) *180/PI;
    *roll = *roll *0,98+rollacc*0.02 ; 
     
/*  double heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(-my, mx);
 
  heading -= DECLINATION * PI / 180;
 
  if (heading > 3.14) heading -= (2 * 3.14);
  else if (heading < -3.14) heading += (2 * 3.14);
 
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;*/
  *pitch *= 180.0 / PI;
  *roll  *= 180.0 / PI;
 
  pc.printf("Pitch, Roll: ");
  pc.printf("%f" , *pitch, 2);
  pc.printf(", ");
  pc.printf("%f" , *roll, 2);
  
  }

// refresh time. set to 500 for part 2 and 50 for part 4
#define REFRESH_TIME_MS 1000

//Init Serial port and LSM9DS1 chip
void setup()
{
    // Use the begin() function to initialize the LSM9DS0 library.
    // You can either call it with no parameters (the easy way):
    uint16_t status = imu.begin();

    //Make sure communication is working
    printf("LSM9DS1 WHO_AM_I's returned: 0x%X\r\n", status);
    printf("Should be 0x6BD\r\n");
}
int main()
{
    setup();  //Setup sensor and Serial
    
    
    imu.begin();
    if (!imu.begin()) {
        printf("Failed to communicate with LSM9DS1.\n");
    }
    imu.calibrate();
    while(1) {
        imu.readTemp();
        imu.readMag();
        imu.readGyro();
 
        pc.printf("gyro: %d %d %d\n\r", imu.gx, imu.gy, imu.gz);
        pc.printf("accel: %d %d %d\n\r", imu.ax, imu.ay, imu.az);
        pc.printf("mag: %d %d %d\n\n\r", imu.mx, imu.my, imu.mz);
        wait(1);
    }
    // printf("------ LSM9DS1 Demo -----------\r\n");
    // printf("Starting now\n");
    // while (true)
    // {
    //     printf("Lets get started");
    //     imu.readAccel();
    //     printf("A: %d, %d, %d\r\n", imu.ax_raw, imu.ay_raw, imu.az_raw);
    //     // imu.readGyro();        
    //     // printf("G: %d, %d, %d\r\n", imu.gx_raw, imu.gy_raw, imu.gz_raw);
    //     // imu.readMag();       
    //     // printf("M: %d, %d, %d\r\n\r\n", imu.mx_raw, imu.my_raw, imu.mz_raw);
    //     // wait_us(REFRESH_TIME_MS);
    // }
}

