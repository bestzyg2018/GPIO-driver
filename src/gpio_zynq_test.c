//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <stdio.h>
//
///* firstdrvtest on
//  * firstdrvtest off
//  */
//int main(int argc, char **argv)
//{
//    int fd;
//    int val = 1;
//    fd = open("/dev/first_gpio", O_RDWR);
//    if (fd < 0)
//    {
//        printf("can't open!\n");
//    }
//    if (argc != 2)
//    {
//        printf("Usage :\n");
//        printf("%s <on|off>\n", argv[0]);
//        return 0;
//    }
//
//    if (strcmp(argv[1], "on") == 0)
//    {
//        val  = 1;
//    }
//    else
//    {
//        val = 0;
//    }
//
//    write(fd, &val, 4);
//    return 0;
//}

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

extern int write();

int main(int argc, char **argv)
{
    int fd,i;
    char par[3];
    fd = open("/dev/ZYNQ-GPIO", O_RDWR);
    if (fd < 0)
    {
        printf("can't open!\n");
    }
    if (argc < 4)
    {
        printf("Usage :\n");
        printf("%s banknum %s bankpin and value\n", argv[0],argv[1]);
        return 0;
    }

    //printf("%s\n banknum %s\n bankpin %s\n value %s\n", argv[0],argv[1],argv[2],argv[3]);


    //convert the char to its integrate mode
    for(i=0;i<3;i++){
        par[i] = atoi(argv[i+1]);
    }

    //write the 3 data to kernel
    write(fd, par, 3);

    return 0;
}

