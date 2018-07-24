#ifndef _LINUX_ELAN_FP_H
#define _LINUX_ELAN_FP_H

#define FINGERPRINT_IOCTL					0x80
#define ID_IOCTL_INIT					_IOW(FINGERPRINT_IOCTL, 0,  int) /* To Get Raw Image (14->8)*/

#define ID_IOCTL_READ_REGISTER				_IOW(FINGERPRINT_IOCTL, 2,  int)
#define ID_IOCTL_WRITE_REGISTER				_IOW(FINGERPRINT_IOCTL, 3,  int)

#define ID_IOCTL_RESET					_IOW(FINGERPRINT_IOCTL, 6,  int)

#define ID_IOCTL_GET_RAW_IMAGE				_IOW(FINGERPRINT_IOCTL, 10, int) /* To Get Raw Image (Original)*/


#define ID_IOCTL_STATUS					_IOW(FINGERPRINT_IOCTL, 12, int)
#define ID_IOCTL_SET_AUTO_RAW_IMAGE			_IOW(FINGERPRINT_IOCTL, 13, int)
#define ID_IOCTL_GET_AUTO_RAW_IMAGE			_IOW(FINGERPRINT_IOCTL, 14, int)
#define ID_IOCTL_READ_CMD				_IOW(FINGERPRINT_IOCTL, 15, int) // General read cmd
#define ID_IOCTL_WRITE_CMD				_IOW(FINGERPRINT_IOCTL, 16, int) // General write cmd
#define ID_IOCTL_IOIRQ_STATUS				_IOW(FINGERPRINT_IOCTL, 17, int) // Use INT to read buffer
#define ID_IOCTL_SPI_STATUS				_IOW(FINGERPRINT_IOCTL, 18, int) // UPdate SPI Speed & CS delay
#define ID_IOCTL_SIG_PID				_IOW(FINGERPRINT_IOCTL, 19, int) // WOE signal event to pid
#define ID_IOCTL_POLL_INIT				_IOW(FINGERPRINT_IOCTL, 20, int) 
#define ID_IOCTL_READ_ALL				_IOW(FINGERPRINT_IOCTL, 21, int) // added v1.441 In IRQ, read all image data, not only one raw.
#define ID_IOCTL_INPUT_KEYCODE			_IOW(FINGERPRINT_IOCTL, 22, int) 
#define ID_IOCTL_POLL_EXIT			_IOW(FINGERPRINT_IOCTL, 23, int)
#define ID_IOCTL_EN_IRQ				_IOW(FINGERPRINT_IOCTL, 55, int)
#define ID_IOCTL_DIS_IRQ			_IOW(FINGERPRINT_IOCTL, 66, int)

#define ELAN_IOCTLID	0xD0	//For customer define
#define IOCTL_SPI_CONFIG _IOW(ELAN_IOCTLID, 8, int)
#define IOCTL_READ_KEY_STATUS _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_WRITE_KEY_STATUS _IOR(ELAN_IOCTLID, 11, int)

#endif /* _LINUX_ELAN_FP_H */

