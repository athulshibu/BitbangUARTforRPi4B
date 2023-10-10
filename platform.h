struct pcdev_platform_data {
	int baudrate;
	int Rx;
	int Tx;	
	char *databuffer;
	int size;
	int perm;
};

#define RDWR 0x11
#define RDONLY 0x01
#define WRONLY 0x10
