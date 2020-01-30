#pragma once
#include <boost/asio.hpp>
#include <RobotRaconteur.h>
#include "robotraconteur_generated.h"
#include "mti2D_RR_interface.h"
//#include "mti2D_RR_interface_stubskel.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <map>

//MTI SDK
#include <atlstr.h>
#include <iostream>
#include "EthernetScanner\EthernetScannerSDK.h"
#include "EthernetScanner\EthernetScannerSDKDefine.h"



using namespace RobotRaconteur;
using namespace boost;
using namespace std;
using namespace mti2D_RR_interface;
namespace pointcloud = com::robotraconteur::pointcloud;
//Global lock to protect from multi-threaded calls
extern boost::recursive_mutex global_lock;

//Class that implements the "Create" object abstract class
//and also use "enable_shared_from_this" for shared_ptr support
class MTI2D_impl : public MTI2D, public pointcloud::sensor::PointCloudSensor_default_impl
{
public:
	MTI2D_impl(string ip, string port);
	
	void Shutdown();	

	~MTI2D_impl();

	virtual pointcloud::PointCloudfPtr Capture();

	

	virtual string getPropertyValue(string propertyName);

	//virtual PointCloudSensorInfoPtr get_point_cloud_sensor_info();

	virtual void setExposureTime(string exposureTime);

	virtual void setAcquisitionLineTime(string acquisitionLineTime);

	virtual void setLaserDeactivated(string isOff);

	virtual void setSignalSelection(string signal);

	virtual void setIsDoubleSampling(string isDoubleSampling);

	bool connected;

	bool isDoubleSampling;
	
	double m_doEthernetScannerBufferX_prev[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	
	double m_doEthernetScannerBufferZ_prev[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	
	int m_iEthernetScannerBufferI_prev[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	
	int m_iEthernetScannerBufferPeakWidth_prev[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];

	virtual void set_point_cloud_sensor_data(PipePtr< pointcloud::sensor::PointCloudSensorDataPtr> value);

private:

	void Disconnect();

	

	uint32_t current_client;

//MTI SDK
	typedef struct _PROGRAMM
	{
		CString FullPath;
		CString DirectoryPath;
		CString ConfigFileName;
	} structProgramm;
	structProgramm m_structProgramm;

#define SCANNER1TIMEOUT 10000

	void Start(const char* ipaddr, const char* nport);

	//virtual RR_SHARED_PTR<LineProfile > get_lineProfile();
	//virtual void set_lineProfile(RR_SHARED_PTR<LineProfile > value);
	
	static DWORD WINAPI ThreadEthernetScannerGetXZI(LPVOID lpParameter);
	HANDLE m_hCEthernetScannerGetXZIThreadID;

	//RR_SHARED_PTR<LineProfile > m_lineProfile;

	BOOL m_bSaveOnce;
	int m_iNumberProfilesToSaveMax;
	int m_iNumberProfilesToSaveCnt;

	void         *m_hEthernetScanner;
	char          m_chEthernetScannerInfo[ETHERNETSCANNER_GETINFOSIZEMAX];
	double        m_doEthernetScannerBufferX[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	double        m_doEthernetScannerBufferZ[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	int           m_iEthernetScannerBufferI[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	int           m_iEthernetScannerBufferPeakWidth[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
	unsigned int  m_uiEthernetScannerEncoder;
	unsigned char m_ucEthernetScannerUSRIO;

	HANDLE m_hCEthernetScannerThreadExitEvent;
	DWORD  m_dwEthernetScanner_Frequenz;
	DWORD  m_dwEthernetScanner_ScanCnt;
	CString m_strScannerIP;
	CString m_strScannerPort;
	CString m_strScannerTimeOut;
	CString m_strCashTime;
	CString m_strPropertyName;

	BOOL m_bViewDisable;
	BOOL m_bRedBackground;
	BOOL m_bGreenBackground;

	unsigned short m_usPicCnt;
	unsigned int m_iPicCntErr;

};

