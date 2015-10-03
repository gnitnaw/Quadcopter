#include <time.h>
#include <stdio.h>
#include <math.h>
//#include <string.h>
#include <unistd.h>
#include <bcm2835.h>
#include <pthread.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>

#include "Setup.h"
//#include "Initialization.h"
#include "Calibration.h"
#include "Device.h"

char Pause(void) {
    char c;
    puts("Start measuring... Press Enter to continue");
    while ( (c=getchar()) != '\n' ){}
    return c;
}

void changemode(int dir)
{
    struct termios oldt, newt;
    if ( dir == 1 ) {
    	tcgetattr( STDIN_FILENO, &oldt);
    	newt = oldt;
    	newt.c_lflag &= ~( ICANON | ECHO );
    	tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }
    else tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}


int kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
}
char waitKey(void);
int power = 1640;
int iPrint = 1;
char waitKey(void) {
    char ch;
    int a;
    while ( !kbhit() ) {
      usleep(1000000);
    }

    iPrint = 0;
    if ( (ch = getchar()) == 'P') {
	puts("Give me the power");
	scanf("%d", &a);
	if ( a<=3280 && a>=1640) power = a;
	printf("Power = %d\n", power);
    }
    iPrint = 1;
    return ch;
}


int main(void) {
    Drone_Status stat;
    int ret;
    char c = 'a';
    if ( (ret=Drone_init(&stat)) != 0 ) return ret;
    puts("Start calibration!");
    Drone_Calibration(&stat);
    Drone_Calibration_printResult(&stat);
    Drone_Start(&stat);

    changemode(1);
//    while ( !kbhit() ) {
//      usleep(1000000);
//    }
    do {
	c = waitKey();
    } while (c != 'c');

//    do {
//        puts("Press c to stop");
//    } while ( (c = getchar()) !='c' );

    Drone_end(&stat);

    return 0;
}

