
#include "mti2D_RR_bridge.h"
namespace pointcloud = com::robotraconteur::pointcloud;
//This program provides an initial Robot Raconteur server for controlling MTI's 2D Laser Profiler (type: 8000-1066-002)
//Functions covers basic operations in this version. 
using namespace RobotRaconteur;
int main(int argc, char* argv[])
{

	if (argc <3)
	{
		cout << "Expected the ip address and port number for the profiler" << endl;
		return -1;
	}

	//Initialize the MTI2D robot object
	RR_SHARED_PTR<MTI2D_impl> mti2d = RR_MAKE_SHARED<MTI2D_impl>(string(argv[1]), string(argv[2]));

	RobotRaconteur::Companion::RegisterStdRobDefServiceTypes();

	if (mti2d->connected == true) {
		ServerNodeSetup node_setup(std::vector<ServiceFactoryPtr>(), "com.robotraconteur.pointcloud.sensor", 2354);
		RobotRaconteurNode::s()->RegisterService("MTI2D", "com.robotraconteur.pointcloud.sensor", mti2d);
		//Create local transport
		/*
		boost::shared_ptr<LocalTransport> t1 = boost::make_shared<LocalTransport>();
		t1->StartServerAsNodeName("mti2D_RR_interface");
		RobotRaconteurNode::s()->RegisterTransport(t1);

		//Initialize the TCP transport and start listenin on port 2354
		boost::shared_ptr<TcpTransport> t = boost::make_shared<TcpTransport>();

		//Attempt to load a TLS certificate
		try
		{
			t->LoadTlsNodeCertificate();
		}
		catch (std::exception&)
		{
			cout << "warning: could not load TLS certificate" << endl;
		}

		t->StartServer(stoi(string(argv[3])));

		//Enable auto-discovery announcement
		t->EnableNodeAnnounce(IPNodeDiscoveryFlags_LINK_LOCAL | IPNodeDiscoveryFlags_SITE_LOCAL | IPNodeDiscoveryFlags_NODE_LOCAL);

		//Register the TCP transport
		RobotRaconteurNode::s()->RegisterTransport(t);

		//Register the MTI2D_RR_interface type so that the node can understand the service definition
		RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<mti2D_RR_interfaceFactory>());

		//Register the MTI2D object as a service so that it can be connected to
		RobotRaconteurNode::s()->RegisterService("MTI2D", "mti2D_RR_interface", mti2d);
		*/
		cout << "MTI2D server started, connect wtih rr+tcp://localhost:" << node_setup.GetTcpTransport()->GetListenPort() << "/?service=MTI2D" << endl;
		//boost::mutex::scoped_lock lock(MTI2D_impl::this_lock);
		//pointcloud::sensor::PointCloudSensor_default_impl::set_point_cloud_sensor_data();
		//this->rrvar_point_cloud_sensor_data = RR_MAKE_SHARED<PipeBroadcaster<pointcloud::sensor::PointCloudSensorDataPtr> >();

		//Stay open until shut down
		cout << "Press enter to quit" << endl;
		getchar();


		//Shutdown
		mti2d->Shutdown();

		//Shutdown the node. This must be called at program exit
		RobotRaconteurNode::s()->Shutdown();

		return 0;
	}
	else {
		cout << "Failed to start MTI2D server." << endl;

		//Stay open until shut down
		cout << "Press enter to quit" << endl;
		return 1;
	}
}


//Class for MTI2D_impl

//Initialize the serial port
MTI2D_impl::MTI2D_impl(string ip, string port) : MTI2D()
{
	connected = false;
	isDoubleSampling = false;
	
	Start(ip.c_str(), port.c_str());
	//Capture();

}


//Shutdown the serial port
void MTI2D_impl::Shutdown()
{

	boost::recursive_mutex::scoped_lock lock(global_lock);
	if (connected)
	{
		Disconnect();
		connected = false;
	}
}

MTI2D_impl::~MTI2D_impl()
{
	Shutdown();
}

void MTI2D_impl::Disconnect() {
	if (m_hEthernetScanner)
	{
		//activate signal to exit the ReadOut-Thread
		SetEvent(m_hCEthernetScannerThreadExitEvent);
		Sleep(1000);
		//close the ethernet connection to the scanner		
		if (m_hCEthernetScannerGetXZIThreadID != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_hCEthernetScannerGetXZIThreadID);
		}
		Sleep(2000);
		EthernetScanner_Disconnect(m_hEthernetScanner);
		m_hEthernetScanner = NULL;
	}
}
/*
PointCloudSensorInfoPtr MTI2D_impl::get_point_cloud_sensor_info() {
	
}
*/
string MTI2D_impl::getPropertyValue(string propertyName)
{
	char chRetBuf[100];
	char *cstr = new char[propertyName.length() + 1];
	strcpy(cstr, propertyName.c_str());
	
	int iRetVal = EthernetScanner_ReadData(m_hEthernetScanner, cstr, chRetBuf, 100, 100);
	if (iRetVal == 1)
	{
		string strTemp(chRetBuf);
		delete[] cstr;
		return strTemp;
	}
	else
	{
		string strTemp = to_string(iRetVal);
		delete[] cstr;
		return strTemp;
	}
}

void MTI2D_impl::setExposureTime(string exposureTime) {

	int exp_t = atoi(exposureTime.c_str());

	if (exp_t>30 && exp_t<2000) {
		string strTemp = "SetExposureTime=" + exposureTime;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setAcquisitionLineTime(string acquisitionLineTime) {

	int acql_t = atoi(acquisitionLineTime.c_str());

	if (acql_t>165 && acql_t<10000) {
		string strTemp = "SetAcquisitionLineTime=" + acquisitionLineTime;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setLaserDeactivated(string isOff) {

	int isOff_i = atoi(isOff.c_str());

	if (isOff_i==0 || isOff_i==1) {
		string strTemp = "SetLaserDeactivated=" + isOff;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setSignalSelection(string signal) {

	int sig_i = atoi(signal.c_str());

	if (sig_i == 0 || sig_i == 1 || sig_i == 2 || sig_i ==3) {
		string strTemp = "SetSignalSelection=" + signal;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setIsDoubleSampling(string isDblSmpling) {

	int isDblSmpling_i = atoi(isDblSmpling.c_str());

	if (isDblSmpling_i == 0 || isDblSmpling_i == 1) {
		isDoubleSampling = isDblSmpling_i;
	}
	
	return;
}


//Capture line scan profile
pointcloud::PointCloudfPtr MTI2D_impl::Capture()
{
	boost::recursive_mutex::scoped_lock lock(global_lock);

	//boost::shared_ptr<LineProfile> profile = boost::make_shared<LineProfile>();
	pointcloud::PointCloudfPtr profile(new pointcloud::PointCloudf());
	//profile->points = AllocateEmptyRRNamedArray<pointcloud::PointCloudPointf>(8192);
	profile->points = AllocateEmptyRRNamedArray<com::robotraconteur::geometryf::Point>(8192);

	if (connected) {

		int iPicCnt = 0;
		//EthernetScanner_GetXZIExtended: the Data are linearized
		int iEthernetScannerScanner1BufferValues = EthernetScanner_GetXZIExtended(this->m_hEthernetScanner,
			this->m_doEthernetScannerBufferX,
			this->m_doEthernetScannerBufferZ,
			this->m_iEthernetScannerBufferI,
			this->m_iEthernetScannerBufferPeakWidth,
			ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX,
			&this->m_uiEthernetScannerEncoder,
			&this->m_ucEthernetScannerUSRIO,
			0,
			NULL,
			0,
			&iPicCnt);

		//pointcloud::PointCloud2Point pointsall[8192];
		//profile->length = iEthernetScannerScanner1BufferValues;
		for (int i = 0; i > iEthernetScannerScanner1BufferValues; i++) {
			//pointcloud::PointCloudPointf pointcloudpoint;
			com::robotraconteur::geometryf::Point pose;
			com::robotraconteur::image::PixelRGBFloatPacked intensity;
			intensity.s.rgb = m_iEthernetScannerBufferI[i];
			pose.s.x = m_doEthernetScannerBufferX[i];
			pose.s.y=m_doEthernetScannerBufferZ[i];
			pose.s.z = 0;
			//pointcloudpoint.s.rgb = intensity;
			//pointcloudpoint.s.point = pose;
			//profile->points->at(i) = pointcloudpoint;
			profile->points->at(i) = pose;
			//pointsall[i] = pointcloudpoint;
		}
		//profile->width = iEthernetScannerScanner1BufferValues;
		//profile->height = 1;
		profile->is_dense = true;
		//profile->points = pointsall;
		//profile->X_data = AttachRRArrayCopy((double*)this->m_doEthernetScannerBufferX, iEthernetScannerScanner1BufferValues);
		//profile->Z_data = AttachRRArrayCopy((double*)this->m_doEthernetScannerBufferZ, iEthernetScannerScanner1BufferValues);
		//profile->I_data = AttachRRArrayCopy((int32_t*)this->m_iEthernetScannerBufferI, iEthernetScannerScanner1BufferValues);


		CString strTemp;
		strTemp.Format("%.4f %.4d %.4f %d",
			this->m_doEthernetScannerBufferX[iEthernetScannerScanner1BufferValues / 2],
			this->m_uiEthernetScannerEncoder,
			this->m_doEthernetScannerBufferZ[iEthernetScannerScanner1BufferValues / 2],
			this->m_iEthernetScannerBufferI[iEthernetScannerScanner1BufferValues / 2]);

		std::cout << strTemp << iPicCnt << iEthernetScannerScanner1BufferValues << std::endl;



		if (iEthernetScannerScanner1BufferValues == ETHERNETSCANNER_ERROR)
		{
			Sleep(1);
			
		}
		else {
			

		}
	}
	pointcloud::sensor::PointCloudSensorDataPtr data(new pointcloud::sensor::PointCloudSensorData());
	data->point_cloud = profile;
	this->rrvar_point_cloud_sensor_data->SendPacket(data);

	return profile;	
}

void MTI2D_impl::set_point_cloud_sensor_data(PipePtr<pointcloud::sensor::PointCloudSensorDataPtr> value) {
	pointcloud::sensor::PointCloudSensor_default_impl::set_point_cloud_sensor_data(value);
	this->rrvar_point_cloud_sensor_data->SetMaxBacklog(3);
}

void MTI2D_impl::Start(const char* ipaddr, const char* nport)
{
	//get the current Path to the exe-File
	char lpPath[1024] = { 0 };
	GetModuleFileName(NULL, lpPath, sizeof(lpPath));
	m_structProgramm.FullPath.Format("%s", lpPath);
	m_structProgramm.DirectoryPath = m_structProgramm.FullPath.Left(m_structProgramm.FullPath.ReverseFind('\\') + 1);

	//get the current version of the DLL 
	unsigned char ucBuffer[1024];
	EthernetScanner_GetVersion(ucBuffer, 1024);
	CString strTemp;
	strTemp.Format("%s", ucBuffer);

	m_hEthernetScanner = NULL;

	//Default-Settings
	m_strScannerIP = ipaddr;
	m_strScannerPort = nport;
	m_strScannerTimeOut.Format("%u", 3000);

	m_strPropertyName = "Temperature";
	m_strCashTime.Format("%u", 100);


	//variables to control the ReadOut-Thread of the Scanner
	m_hCEthernetScannerThreadExitEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_hCEthernetScannerGetXZIThreadID = INVALID_HANDLE_VALUE;

	//initialize the view-class to show the scans


	m_dwEthernetScanner_ScanCnt = 0;

	m_iNumberProfilesToSaveCnt = 0;
	m_iNumberProfilesToSaveMax = 1;

	if (m_hEthernetScanner == NULL)
	{
		m_iPicCntErr = 0;
		CString strTemp;
		//start the connection to the Scanner
		m_hEthernetScanner = EthernetScanner_Connect(m_strScannerIP.GetBuffer(0),
			m_strScannerPort.GetBuffer(0),
			atoi(m_strScannerTimeOut));
		//if no valid value: ERROR
		if (m_hEthernetScanner == NULL)
		{
			std::cout << "Error: EthernetScanner_Connect: NULL-Pointer";
			std::cin.get();
			return;
		}

		//Wait for valid connection to scanner
		DWORD dwTimeOut = GetTickCount();
		int uiConnectionStatus = 0;

		while (uiConnectionStatus != ETHERNETSCANNER_TCPSCANNERCONNECTED)
		{
			//current state of the connection
			EthernetScanner_GetConnectStatus(m_hEthernetScanner, &uiConnectionStatus);
			//Detect the timeout
			if (GetTickCount() - dwTimeOut > 3000)
			{
				std::cout << "Error: EthernetScanner_Connect: Error in connection";
				m_hEthernetScanner = EthernetScanner_Disconnect(m_hEthernetScanner);
				std::cin.get();
				return;
			}
		}

		int iRes = -1;
		dwTimeOut = GetTickCount();
		while (GetTickCount() - dwTimeOut < SCANNER1TIMEOUT)
		{
			//the linearization table need to be transfered from the Scanner to the Host.
			//it takes some times
			//to be sure that the linearization table was received runs the "EthernetScanner_GetInfo"-function more times
			iRes = EthernetScanner_GetInfo(m_hEthernetScanner, m_chEthernetScannerInfo, ETHERNETSCANNER_GETINFOSIZEMAX, "text");
			if (iRes > 0)
			{
				break;
			}

		}
		if (iRes < 0)
		{
			std::cout << "Error in linearisation data!!!";
			std::cin.get();
		}
		iRes = EthernetScanner_GetInfo(m_hEthernetScanner, m_chEthernetScannerInfo, ETHERNETSCANNER_GETINFOSIZEMAX, "xml");
		if (iRes > 0)
		{		
			connected = true;
			// m_lineProfile = RR_MAKE_SHARED<LineProfile>();

			//start the ReadOut-Thread
			DWORD lpThreadId;
			m_hCEthernetScannerGetXZIThreadID = CreateThread(NULL,
				NULL,
				(LPTHREAD_START_ROUTINE)ThreadEthernetScannerGetXZI,
				(LPVOID)this, //Pointer to this Class. to access the Class-Variables from the Thread
				0,
				&lpThreadId);
			if (m_hCEthernetScannerGetXZIThreadID)
			{
			}
		}
		else
		{
			//get XML Error
		}

		/*std::cout << "Reached the end of 'Start', enter to continue" << std::endl;
		std::cin.get();*/
	}
}

/*
ReadOut-Thread of Scans
runs til the m_hCEthernetScannerScanner1ThreadExitEvent-Event not signaled
*/
DWORD WINAPI MTI2D_impl::ThreadEthernetScannerGetXZI(LPVOID lpParameter)
{
	MTI2D_impl *pApp = (MTI2D_impl *)lpParameter;

	bool bRun = TRUE;

	DWORD dwScanner1FrequnzTimer = GetTickCount();
	int iScanner1ShowFrequency = GetTickCount();
	int iScanner1ShowScan = GetTickCount();

	CString strTemp;

	bool bShowOnce = true;// false;
	int uiConnectionStatus = ETHERNETSCANNER_TCPSCANNERDISCONNECTED;

	int iFrameCntTemp = 0;

	/*std::cout << "Entering infinite loop, enter to continue" << std::endl;
	std::cin.get();*/

	//chech the valid value of the Scanner-Handle: NULL is not valid, otherwise is OK
	while (pApp->m_hEthernetScanner)
	{
		//current state of the connection
		EthernetScanner_GetConnectStatus(pApp->m_hEthernetScanner, &uiConnectionStatus);

		if (uiConnectionStatus == ETHERNETSCANNER_TCPSCANNERCONNECTED)
		{

			int iPicCnt = 0;
			//EthernetScanner_GetXZIExtended: the Data are linearized
			int len_arr = ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX;
			for (int i = 0; i < len_arr; i++) {
				pApp->m_doEthernetScannerBufferX_prev[i] = pApp->m_doEthernetScannerBufferX[i];
				pApp->m_doEthernetScannerBufferZ_prev[i] = pApp->m_doEthernetScannerBufferZ[i];
				pApp->m_iEthernetScannerBufferI_prev[i] = pApp->m_iEthernetScannerBufferI[i];
			}
			int iEthernetScannerScanner1BufferValues = EthernetScanner_GetXZIExtended(pApp->m_hEthernetScanner,
				pApp->m_doEthernetScannerBufferX,
				pApp->m_doEthernetScannerBufferZ,
				pApp->m_iEthernetScannerBufferI,
				pApp->m_iEthernetScannerBufferPeakWidth,
				ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX,
				&pApp->m_uiEthernetScannerEncoder,
				&pApp->m_ucEthernetScannerUSRIO,
				0,
				NULL,
				0,
				&iPicCnt);

			if (iEthernetScannerScanner1BufferValues == ETHERNETSCANNER_ERROR)
			{
				Sleep(1);
				continue;
			}

			if (iPicCnt - pApp->m_usPicCnt != 1)
			{
				if ((iPicCnt == 0) && (pApp->m_usPicCnt == 0xFFFF))
				{
				}
				else
				{
					pApp->m_iPicCntErr++;
				}
			}
			pApp->m_usPicCnt = iPicCnt;
			pointcloud::PointCloudfPtr profile(new pointcloud::PointCloudf());
			profile->points = AllocateEmptyRRNamedArray<com::robotraconteur::geometryf::Point>(8192);
			//RR_SHARED_PTR <LineProfile> profile = RR_MAKE_SHARED<LineProfile>();
			
			if (pApp->isDoubleSampling) {
				double m_doEthernetScannerBufferX_temp[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];

				double m_doEthernetScannerBufferZ_temp[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];

				int m_iEthernetScannerBufferI_temp[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];				

				for (int i = 0; i < len_arr; i++) {
					m_doEthernetScannerBufferX_temp[i] = (pApp->m_doEthernetScannerBufferX_prev[i] + pApp->m_doEthernetScannerBufferX[i])/2.0;
					m_doEthernetScannerBufferZ_temp[i] = (pApp->m_doEthernetScannerBufferZ_prev[i] + pApp->m_doEthernetScannerBufferZ[i])/2.0;
					m_iEthernetScannerBufferI_temp[i] = (pApp->m_iEthernetScannerBufferI_prev[i] + pApp->m_iEthernetScannerBufferI[i])/2;
				}

				for (int i = 0; i > iEthernetScannerScanner1BufferValues; i++) {
					//pointcloud::PointCloud2Point pointcloudpoint;
					com::robotraconteur::geometryf::Point pose;
					com::robotraconteur::image::PixelRGBFloatPacked intensity;
					intensity.s.rgb = m_iEthernetScannerBufferI_temp[i];
					pose.s.x = m_doEthernetScannerBufferX_temp[i];
					pose.s.y = m_doEthernetScannerBufferZ_temp[i];
					pose.s.z = 0;
					//pointcloudpoint.s.rgb = intensity;
					//pointcloudpoint.s.point = pose;
					profile->points->at(i) = pose;
					//pointsall[i] = pointcloudpoint;
				}
				//profile->width = iEthernetScannerScanner1BufferValues;
				//profile->height = 1;
				profile->is_dense = true;
			}
			else {
				
				for (int i = 0; i > iEthernetScannerScanner1BufferValues; i++) {
					//pointcloud::PointCloud2Point pointcloudpoint;
					com::robotraconteur::geometryf::Point pose;
					com::robotraconteur::image::PixelRGBFloatPacked intensity;
					intensity.s.rgb = pApp->m_iEthernetScannerBufferI[i];
					pose.s.x = pApp->m_doEthernetScannerBufferX[i];
					pose.s.y = pApp->m_doEthernetScannerBufferZ[i];
					pose.s.z = 0;
					//pointcloudpoint.s.rgb = intensity;
					//pointcloudpoint.s.point = pose;
					profile->points->at(i) = pose;
					//pointsall[i] = pointcloudpoint;
				}
				//profile->width = iEthernetScannerScanner1BufferValues;
				//profile->height = 1;
				profile->is_dense = true;
			}
			
			//set_point_cloud_sensor_data->SendPacket(profile);
			//pApp->set_lineProfile(profile);

			//if the scan data was received: do anything
			if (iEthernetScannerScanner1BufferValues)
			{
				///count the frequency
				pApp->m_dwEthernetScanner_Frequenz++;
				pApp->m_dwEthernetScanner_ScanCnt++;

				//with higher frequency or slower PC-CPU the view of scans can be disabled
				if (pApp->m_bViewDisable == false)
				{
					//decrease or disable the view of the scans if the scanner frequency go high!
					if (GetTickCount() - iScanner1ShowScan > 100)
					{
						iScanner1ShowScan = GetTickCount();

					}
				}
			}
		}
		//exit the thread if the signal comes
		if (WaitForSingleObject(pApp->m_hCEthernetScannerThreadExitEvent, 0) == WAIT_OBJECT_0)
		{
			bRun = FALSE;
		}

	}
	return (0);
}


/*
//Set the lineProfile property
void MTI2D_impl::set_lineProfile(RR_SHARED_PTR<LineProfile > profile)
{
	m_lineProfile->length = profile->length;
	m_lineProfile->X_data = profile->X_data;
	m_lineProfile->Z_data = profile->Z_data;
	m_lineProfile->I_data = profile->I_data;	
}

//Return the DistanceTraveled property
RR_SHARED_PTR<LineProfile > MTI2D_impl::get_lineProfile()
{
	return m_lineProfile;
}
*/

boost::recursive_mutex global_lock;