/**********************************************************
 SPI_Hello_Arduino
   Configures a Raspberry Pi as an SPI master and  
   demonstrates bidirectional communication with an 
   Arduino Slave.
   
 Compile String:
 g++ -o SPI_Hello_Arduino hello_ardu.cpp
***********************************************************/

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>

using namespace std;

int fd;
unsigned char hello[] = {'H','e','l','l','o',' ',
                         'A','r','d','u','i','n','o','\n'};
unsigned char result;

int spiTxRx(unsigned char txDat);

int main(void)
{
  fd = open("/dev/spidev0.0", O_RDWR);
  if (fd < 0) {
    perror("Failed to open /dev/spidev0.0");
    return 1;
  }

  // Set SPI mode to Mode 0 (CPOL=0, CPHA=0)
  uint8_t mode = SPI_MODE_0;
  ioctl(fd, SPI_IOC_WR_MODE, &mode);

  // Set clock speed to 500 kHz (safe speed for 16MHz Arduino slave)
  unsigned int speed = 500000;
  ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  while (1)
  {
    for (size_t i = 0; i < sizeof(hello); i++)
    {
      result = spiTxRx(hello[i]);
      cout << result << flush;
      usleep(100); // 100 µs between bytes gives Arduino time to handle SPDR
    }

    // Give Arduino Serial Monitor time to clear its output buffer
    usleep(50000); // 50 ms pause between messages
  }

  close(fd);
  return 0;
}

int spiTxRx(unsigned char txDat)
{
  unsigned char rxDat = 0;
  struct spi_ioc_transfer spi;
  memset(&spi, 0, sizeof(spi));

  spi.tx_buf = (unsigned long)&txDat;
  spi.rx_buf = (unsigned long)&rxDat;
  spi.len    = 1;

  ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
  return rxDat;
}