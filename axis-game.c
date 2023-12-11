//*****************************************************************************
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "drivers/buttons.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"

//*****************************************************************************
//
// Constants
//
//*****************************************************************************

#define APP_NAME                "LAB 8"
#define G1VAL_MSS               9.8
#define DELAY                   100
#define MPU9150_I2C_ADDRESS     0x69
#define DELAY_SPLASH            10000000
#define NORM                    16
#define MOVED_THRESH            0.5

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

  char buf[40];
  static int index = 0;
  static float xArray[NORM] = {0.0};
  static float yArray[NORM] = {0.0};
  static float zArray[NORM] = {0.0};
  float xAvg, yAvg, zAvg;
  float moved;
  int32_t xPos, yPos;
  int32_t xWide, yHigh;
  float xMult, yMult;
  bool gDone = false;
  bool gButtonsEnabled = true;
  bool g_bInstantDisplay = false;
  tI2CMInstance g_sI2CInst;
  tMPU9150 g_sMPU9150;
  int score = 0;
  int a;
  int b;
  int count = 30;

// A boolean that is set when a MPU9150 command has completed.
volatile bool g_bMPU9150Done = false;

volatile uint8_t g_vui8ErrorFlag = 0;

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
int StringToUart(char *str1);
int clearBackground(int);
void MPU9150Example(void);
void process_input(void);
int init_mpu9150(void);
void MPU9150Itteration(void);

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
tContext g_sContext;
tRectangle sRect;
uint32_t in_char;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//TIMER
void
Timer0IntHandler(void){
  
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  count--;
  
}


//*****************************************************************************
//
// Converts float to String
//
//*****************************************************************************
char *floatToString(float x)
{
  static char buf[32];
  int mantissa = (int)(x);
  int fraction = abs((int)((x - mantissa)*1000));
  if (fraction > 99)
  {
    sprintf(buf, "%d.%d    ", mantissa, fraction);
  } 
  else if (fraction > 9)
  {
    sprintf(buf, "%d.0%d    ", mantissa, fraction);
  }
  else
  {    
    sprintf(buf, "%d.00%d    ", mantissa, fraction);
  }
  
  return buf;
}

//*****************************************************************************
//
// Send characters using UART 
//
//*****************************************************************************

void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
      // Write the next character to the UART.
      UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// print menu
//
//*****************************************************************************

void
printMenu(void)
{
 UARTSend("I - Toggle numeric display \r\n", 31);
 UARTSend("M - Print this Menu \r\n", 24);
 UARTSend("Q - Quit Program \r\n", 20);
}

//*****************************************************************************
//
// process Input
//
//*****************************************************************************

void
process_input(void)
{
    switch (tolower(in_char))
    {
      case 'i':
        if (g_bInstantDisplay==true)
          g_bInstantDisplay=false;
        else
          g_bInstantDisplay=true;

          
      break;
      case 'q':
        gDone = true;
      break;
      case 'm':
        printMenu();
      break;
    } 
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************

void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    UARTIntClear(UART0_BASE, ui32Status);
 
      in_char = UARTCharGetNonBlocking(UART0_BASE);
      process_input();
     

}

//*****************************************************************************
//
// Initalize the UART.
//
//*****************************************************************************

int init_UART(void)
{
   
    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   
    // Set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART0_BASE, UARTIntHandler);

    return (0);
}

//*****************************************************************************
//
// init_OLED() function
//
//*****************************************************************************

int init_OLED()
{
 
    // Initialize the display driver.
    CFAL96x64x16Init();

    // Initialize the graphics context.
    GrContextInit(&g_sContext, &g_sCFAL96x64x16);

    // Fill the top part of the screen with blue to create the banner.
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.i16YMax = 9;
    GrContextForegroundSet(&g_sContext, ClrSpringGreen);
    GrRectFill(&g_sContext, &sRect);

    // Put the application name in the middle of the banner.
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrContextFontSet(&g_sContext, g_psFontFixed6x8);
    GrStringDrawCentered(&g_sContext, APP_NAME, -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 4, 0);
    
    // Initialize the display and write some instructions.
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrStringDrawCentered(&g_sContext, "Connect a", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 20, false);
    GrStringDrawCentered(&g_sContext, "terminal", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 30, false);
    GrStringDrawCentered(&g_sContext, "to UART0.", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 40, false);
    GrStringDrawCentered(&g_sContext, "115000,N,8,1", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 50, false);
    
  return 0;
}

//*****************************************************************************
//
// clearBackground() function
//
//*****************************************************************************

int clearBackground(int yStart)
{
  sRect.i16XMin = 0;
  sRect.i16YMin = yStart;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext) - 1;
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext) - 1;;
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &sRect);        
  GrContextForegroundSet(&g_sContext, ClrWhite);
  return(0);
}

//*****************************************************************************
//
// Example of using pushbutton and analog comparator interrupts
//
//*****************************************************************************
int
main(void)
{     
    // Enable lazy stacking for interrupt handlers
    FPULazyStackingEnable();
      
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // Disable Interrupts
    IntMasterDisable(); 
  
    // Init OLED
    init_OLED();
     
    // Initialize UART
    init_UART();
    
    //TIMER
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
  
    // Wait for Splash Display
    SysCtlDelay(DELAY_SPLASH);
    
    
    // clear the screen
    clearBackground(0);
  
    // Show menu options
    printMenu();
    
    xWide = GrContextDpyWidthGet(&g_sContext);
    yHigh = GrContextDpyHeightGet(&g_sContext);
    
    a = (rand()%(xWide - 30) + 15);
    b = (rand()%(yHigh - 20) + 15);
  
    // Enable processor interrupts.
    IntMasterEnable();
   
    // Enable & Configure MPU9150
    init_mpu9150();
    
    // Loop pseudo-forever 
    while(!gDone)
    {
      // Perform MPU-9150 operations
      MPU9150Itteration();
      
    
      // Delay for a bit.
      SysCtlDelay(DELAY);      
    }
    
    UARTSend((uint8_t *)"Goodbye...", 10);
} // main()


//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
AccelCrossMPU9150Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
  // If the transaction succeeded set the data flag to indicate to
  // application that this transaction is complete and data may be ready.
  if(ui8Status == I2CM_STATUS_SUCCESS)
  {
    // Indicate that the MPU9150 transaction has completed.
    g_bMPU9150Done = true;
  }
  // Store the most recent status in case it was an error condition
  g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// The MPU9150 io initialization DK-TM4C123G specific
//
//*****************************************************************************

int init_mpu9150(void)
{
  // Initialize I2C3 Peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlDelay(100);
  SysCtlPeripheralReset(I2C3_BASE);
  GPIOPinConfigure(GPIO_PD0_I2C3SCL);
  GPIOPinConfigure(GPIO_PD1_I2C3SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
  GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
  
  // Enable the supplied I2C Base Clock
  I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);

  // Enable supplied I2C Base Master Block
  I2CMasterEnable(I2C3_BASE);
  
  // Initialize the I2C instance associated with I2C3
  I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, SysCtlClockGet());
  
  // Initialize the MPU9150. This code assumes that the I2C master instance
  // has already been initialized.

  g_bMPU9150Done = false;
  MPU9150Init(&g_sMPU9150, &g_sI2CInst, MPU9150_I2C_ADDRESS, AccelCrossMPU9150Callback, 0);
  while(!g_bMPU9150Done)
    {
    }

  // Configure the MPU9150 for +/- 4 g accelerometer range.

  g_bMPU9150Done = false;
  MPU9150ReadModifyWrite(&g_sMPU9150, MPU9150_O_ACCEL_CONFIG,
                         ~MPU9150_ACCEL_CONFIG_AFS_SEL_M,
                         MPU9150_ACCEL_CONFIG_AFS_SEL_4G, AccelCrossMPU9150Callback,
                         0);
  while(!g_bMPU9150Done)
  {
  }
  
  return 0;
}

//*****************************************************************************
//
// The MPU9150 example.
//
//*****************************************************************************
void
MPU9150Itteration(void)
{
   
  // Request reading from the MPU9150.
  while(index < NORM){
    g_bMPU9150Done = false;
    MPU9150DataRead(&g_sMPU9150, AccelCrossMPU9150Callback, 0);
    while(!g_bMPU9150Done)
    {
    }
    

    MPU9150DataAccelGetFloat(&g_sMPU9150, &xArray[index], &yArray[index],
    &zArray[index]);
    
    xAvg = (xAvg + xArray[index]);
    yAvg = (yAvg + yArray[index]);
    zAvg = (zAvg + zArray[index]);
    
    index++;
  }
  
  index = 0;
 
  // Generate Averages over last NORM samples
  
  xAvg = xAvg/NORM;
  yAvg = yAvg/NORM;
  zAvg = zAvg/NORM;
  
  
if(g_bInstantDisplay == false){
  // Display Numerical Data
    
  //sprintf(buf, "  x = %.2f  ", xAvg);
  GrStringDrawCentered(&g_sContext, floatToString(xAvg), -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 20, true);
  //sprintf(buf, "  y = %.2f  ", yAvg);
  GrStringDrawCentered(&g_sContext, floatToString(yAvg), -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 30, true);
  //sprintf(buf, "  z = %.2f  ", zAvg);
  GrStringDrawCentered(&g_sContext, floatToString(zAvg), -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 40, true);
 
  }


 if(g_bInstantDisplay == true){
  // Bubble Display And Game Features
  sRect.i16XMin = 0;
  sRect.i16YMin = 15;
  sRect.i16XMax = GrContextDpyWidthGet(&g_sContext) - 1;
  sRect.i16YMax = GrContextDpyHeightGet(&g_sContext) - 1;;
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &sRect);        
  GrContextForegroundSet(&g_sContext, ClrWhite);
  
    int x1 = a;
    int x2 = a + 5;
    int y1 = b;
    int y2 = b + 5;
    int x3 = a + 5;
    int x4 = a;
    int y3 = b;
    int y4 = b + 5;
    GrLineDraw(&g_sContext, x1, y1, x2, y2);
    GrLineDraw(&g_sContext, x3, y3, x4, y4);
 
  
  xPos = (yAvg * xMult) + (GrContextDpyWidthGet(&g_sContext) /2);
  yPos = ((xAvg * yMult) + (GrContextDpyHeightGet(&g_sContext) / 2) + 20);

  
  GrCircleDraw(&g_sContext, xPos, yPos, 5);
  
  
  sprintf(buf, "score:%d time:%d ", score, count);
  GrStringDrawCentered(&g_sContext, buf, -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 10, true);
  
  if(abs((xPos - a)) <= 3 && abs((yPos - b)) <= 3){
    score++;
    
    a = (rand()%(xWide - 30) + 15);
    b = (rand()%(yHigh - 20) + 15);
    
  }
  
  if(count <= 0){
    clearBackground(0);
    
    GrStringDrawCentered(&g_sContext,"GAME OVER", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 10, true);
    sprintf(buf, "SCORE %d", score);
    GrStringDrawCentered(&g_sContext,buf, -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 30, true);
    GrStringDrawCentered(&g_sContext,"PRESS RESTART", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 50, true);
    
    gDone = true;
    
  }
   
  
    xMult = ((float)xWide / 2) / G1VAL_MSS;
    yMult = ((float)yHigh / 2) / G1VAL_MSS;
   
 }

    
    
 
}      

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    
    I2CMIntHandler(&g_sI2CInst);
}
