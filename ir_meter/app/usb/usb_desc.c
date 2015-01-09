/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_desc.c
* Author             : MCD Application Team
* Version            : V2.2.1
* Date               : 09/22/2008
* Description        : Descriptors for Custom HID Demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const u8 CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength �豸���������ֽ�����С*/
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType �豸���������ͱ��*/
    0x10,                       /*bcdUSB USB�汾��01.10*/
    0x01,
    0x00,                       /*bDeviceClass USB������豸�����*/
    0x00,                       /*bDeviceSubClass USB������������*/
    0x00,                       /*bDeviceProtocol USB������豸Э�����*/
    0x40,                       /*bMaxPacketSize40 �˵�0��������С*/
    0x83,                       /*idVendor (0x0483) ���̱��*/
    0x04,
    0x34,                       /*idProduct = 0x1234 ��Ʒ���*/
    0x12,
    0x00,                       /*bcdDevice rel. 1.00 �豸�������*/
    0x01,
    1,                          /*Index of string descriptor describing 
                                              manufacturer �豸�����ַ���������*/
    2,                          /*Index of string descriptor describing
                                             product ������Ʒ�ַ���������*/
    3,                          /*Index of string descriptor describing the
                                             device serial number �����豸���к��ַ���������*/
    0x01                        /*bNumConfigurations ���ܵ���������*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const u8 CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuation Descriptor size �������������ֽ�����С*/
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration �������������ͱ��*/
    CUSTOMHID_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned �����÷��ص��������ݴ�С*/
    0x00,
    0x01,         /* bNumInterfaces: 1 interface ��������֧�ֵĽӿ�����*/
    0x01,         /* bConfigurationValue: Configuration value Set_Configuration��������Ҫ�Ĳ���ֵ*/
    0x00,         /* iConfiguration: Index of string descriptor describing
                                 the configuration ���������õ��ַ���������ֵ*/
    0xC0,         /* bmAttributes: Bus powered ����ģʽ��ѡ��*/
    0x32,         /* MaxPower 300 mA: this current is used for detecting Vbus �豸��������ȡ��������*/

    /************** Descriptor of Custom HID interface ****************/
    /* 09 */
    0x09,         /* bLength: Interface Descriptor size �ӿ����������ֽ�����С*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type �ӿ������������ͱ��*/
    0x00,         /* bInterfaceNumber: Number of Interface �ýӿڵı��*/
    0x00,         /* bAlternateSetting: Alternate setting ���õĽӿ����������*/
    0x02,         /* bNumEndpoints �ýӿ�ʹ�õĶ˵������������˵�0*/
    0x03,         /* bInterfaceClass: HID �ӿ�����*/
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot �ӿ�������*/
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse �ӿ���ѭ��Э��*/
    0,            /* iInterface: Index of string descriptor �����ýӿڵ��ַ�������ֵ*/
    /******************** Descriptor of Custom HID HID ********************/
    /* 18 */
    0x09,         /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType */
    CUSTOMHID_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 27 */
    0x07,          /* bLength: Endpoint Descriptor size �˵��������ֽ�����С*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: �˵����������ͱ��*/

    0x81,          /* bEndpointAddress: Endpoint Address (IN) �˵��ַ�������������*/
    0x03,          /* bmAttributes: Interrupt endpoint �˵�Ĵ�����������*/
    0x40,          /* wMaxPacketSize: 64 Bytes max �˵��ա�����������С*/
    0x00,
    0x0A,          /* bInterval: Polling Interval (10 ms) */
    /* 34 */
    /******************** Descriptor of  endpoint ********************/
    /* 27 */
    0x07,	/* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
			/*	Endpoint descriptor type */
    0x02,	/* bEndpointAddress: *//*	Endpoint Address (OUT) */		
    0x03,	/* bmAttributes: Interrupt endpoint */
    0x40,	/* wMaxPacketSize: 2 Bytes max  */
    0x00,
    0x0A,	/* bInterval: Polling Interval (10 ms) */
    /* 41 */
  }
  ; /* CustomHID_ConfigDescriptor */
const u8 CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
  {
    0x05, 0xFF,                    // USAGE_PAGE(User define)
    0x09, 0xFF,                    // USAGE(User define)
    0xa1, 0x01,                    // COLLECTION (Application)

    0x05, 0x01,                    // USAGE_PAGE(1)
    0x19, 0x00,                    //   USAGE_MINIMUM(0)
    0x29, 0xFF,                    //   USAGE_MAXIMUM(255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,                    //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x40,                    //   REPORT_COUNT (64)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

    0x05, 0x02,                    // USAGE_PAGE(2)
    0x19, 0x00,                    //   USAGE_MINIMUM (0)
    0x29, 0xFF,                    //   USAGE_MAXIMUM (255)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,                    //   LOGICAL_MAXIMUM (255)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x75, 0x40,                    //   REPORT_SIZE (64)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

    0xc0                           // END_COLLECTION
  }; /* CustomHID_ReportDescriptor */

/* USB String Descriptors (optional) */
const u8 CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const u8 CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };

const u8 CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'C', 0,
    'u', 0, 's', 0, 't', 0, 'm', 0, ' ', 0, 'H', 0, 'I', 0,
    'D', 0
  };
u8 CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0,'3', 0,'2', 0, '1', 0, '0', 0
  };

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

